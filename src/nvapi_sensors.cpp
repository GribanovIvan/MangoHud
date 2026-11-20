#include "nvapi_sensors.h"
#include <spdlog/spdlog.h>

#include <cstdio>
#include <cstring>
#include <dlfcn.h>

namespace {

const char* NVAPI_LIBRARY = "libnvidia-api.so.1";

// NVAPI has no exported symbols beyond nvapi_QueryInterface; every entry point
// is looked up by a numeric id.
const uint32_t QUERY_INITIALIZE = 0x0150e828;
const uint32_t QUERY_UNLOAD = 0xd22bdd7e;
const uint32_t QUERY_ENUM_PHYSICAL_GPUS = 0xe5ac921f;
const uint32_t QUERY_GET_BUS_ID = 0x1be0b8e5;
const uint32_t QUERY_THERMALS = 0x65fe3aad;
const uint32_t QUERY_VOLTAGE = 0x465f9bcf;

const int MAX_PHYSICAL_GPUS = 64;

// Thermal values are fixed point with 8 fractional bits.
const int FIXED_POINT_SCALE = 256;

const int MICROVOLTS_PER_MILLIVOLT = 1000;
const int MAX_PLAUSIBLE_MILLIVOLTS = 2000;

// Number of sensor slots NVAPI returns for the accepted mask.
const int THERMAL_SLOTS = 40;

} // namespace

NvApiSensors::~NvApiSensors() {
    if (library_) {
        if (query_interface_) {
            typedef int32_t (*unload_fn)(void);
            unload_fn unload = (unload_fn)query_interface_(QUERY_UNLOAD);
            if (unload)
                unload();
        }
        dlclose(library_);
    }
}

bool NvApiSensors::find_gpu(unsigned bus) {
    typedef int32_t (*enum_fn)(void**, uint32_t*);
    typedef int32_t (*bus_fn)(void*, uint32_t*);

    enum_fn enum_gpus = (enum_fn)query_interface_(QUERY_ENUM_PHYSICAL_GPUS);
    bus_fn get_bus = (bus_fn)query_interface_(QUERY_GET_BUS_ID);
    if (!enum_gpus || !get_bus)
        return false;

    void* handles[MAX_PHYSICAL_GPUS];
    memset(handles, 0, sizeof(handles));
    uint32_t count = 0;

    if (enum_gpus(handles, &count) != 0)
        return false;

    for (uint32_t i = 0; i < count && i < MAX_PHYSICAL_GPUS; i++) {
        uint32_t id = 0;
        if (get_bus(handles[i], &id) == 0 && id == bus) {
            gpu_ = handles[i];
            return true;
        }
    }

    return false;
}

bool NvApiSensors::calculate_mask() {
    // The sensor mask this GPU accepts is not reported anywhere, so widen it one
    // bit at a time until the call is rejected and keep the last accepted value.
    for (int bit = 0; bit < 32; bit++) {
        Thermals probe;
        memset(&probe, 0, sizeof(probe));
        probe.version = (uint32_t)(sizeof(Thermals) | (2 << 16));
        const uint32_t probe_mask = 1u << bit;
        probe.mask = (int32_t)probe_mask;

        if (get_thermals_(gpu_, &probe) != 0) {
            mask_ = (int32_t)(probe_mask - 1u);
            return mask_ != 0;
        }
    }

    return false;
}

bool NvApiSensors::init(const char* pci_bus_id) {
    unsigned domain, bus, device, function;

    if (!pci_bus_id ||
        sscanf(pci_bus_id, "%x:%x:%x.%x", &domain, &bus, &device, &function) != 4) {
        SPDLOG_DEBUG("NvAPI: could not parse PCI bus id");
        return false;
    }

    library_ = dlopen(NVAPI_LIBRARY, RTLD_NOW);
    if (!library_) {
        SPDLOG_DEBUG("NvAPI: {} not available, extra sensors disabled", NVAPI_LIBRARY);
        return false;
    }

    query_interface_ = (void* (*)(uint32_t))dlsym(library_, "nvapi_QueryInterface");
    if (!query_interface_) {
        SPDLOG_DEBUG("NvAPI: nvapi_QueryInterface missing");
        return false;
    }

    typedef int32_t (*init_fn)(void);
    init_fn initialize = (init_fn)query_interface_(QUERY_INITIALIZE);
    if (!initialize || initialize() != 0) {
        SPDLOG_DEBUG("NvAPI: initialization failed");
        return false;
    }

    get_thermals_ = (int32_t (*)(void*, Thermals*))query_interface_(QUERY_THERMALS);
    if (!get_thermals_) {
        SPDLOG_DEBUG("NvAPI: thermals interface missing");
        return false;
    }

    // Optional: absent voltage support must not disable the temperatures.
    get_voltage_ = (int32_t (*)(void*, Voltage*))query_interface_(QUERY_VOLTAGE);

    if (!find_gpu(bus)) {
        SPDLOG_DEBUG("NvAPI: no GPU matching bus {:#x}", bus);
        return false;
    }

    if (!calculate_mask()) {
        SPDLOG_DEBUG("NvAPI: could not determine sensor mask");
        return false;
    }

    base_ = detect_base();

    available_ = true;
    SPDLOG_DEBUG("NvAPI: sensors available (mask {:#x}, base {}, voltage {})",
                 mask_, base_, get_voltage_ ? "yes" : "no");
    return true;
}

// One NVAPI round trip fills the whole sensor array for the accepted mask.
int NvApiSensors::read_thermals(int* out) {
    Thermals thermals;
    memset(&thermals, 0, sizeof(thermals));
    thermals.version = (uint32_t)(sizeof(Thermals) | (2 << 16));
    thermals.mask = mask_;

    if (get_thermals_(gpu_, &thermals) != 0)
        return -1;

    for (int i = 0; i < THERMAL_SLOTS; i++)
        out[i] = thermals.values[i] / FIXED_POINT_SCALE;

    return 0;
}

int NvApiSensors::read_sensor(int index) {
    if (!available_ || index < 0 || index >= THERMAL_SLOTS)
        return -1;

    int values[THERMAL_SLOTS];
    if (read_thermals(values) != 0)
        return -1;

    int value = values[index];

    // An absent sensor reads back as zero; a plausible die never does.
    if (value <= 0 || value >= 255)
        return -1;

    return value;
}

// NVAPI writes the accepted sensors starting at a part-specific index, so a slot
// named by constant moves between architectures. Find the first populated index.
int NvApiSensors::detect_base() {
    int values[THERMAL_SLOTS];
    if (read_thermals(values) != 0)
        return -1;

    for (int i = 0; i < THERMAL_SLOTS; i++)
        if (values[i] > 0 && values[i] < 255)
            return i;

    return -1;
}

// Try the index verified on GA107 first, then the placement derived from this
// part's base: whichever answers is the memory die on this card.
int NvApiSensors::memory_index() {
    int candidates[] = { VRAM_INDEX, base_ + VRAM_INDEX_OFFSET,
                         base_ + VRAM_INDEX_OFFSET - 1 };

    for (int candidate : candidates) {
        int value = read_sensor(candidate);
        if (value >= 0)
            return candidate;
    }

    return VRAM_INDEX;
}

int NvApiSensors::hotspot() {
    int value = read_sensor(HOTSPOT_INDEX);

    // Verified on GA107 and AD102; other parts may place it one slot along.
    if (value < 0 && base_ >= 0)
        value = read_sensor(base_ + 1);

    return value;
}

int NvApiSensors::vram() {
    return read_sensor(memory_index());
}

int NvApiSensors::voltage() {
    if (!available_ || !get_voltage_)
        return -1;

    Voltage voltage;
    memset(&voltage, 0, sizeof(voltage));
    voltage.version = (uint32_t)(sizeof(Voltage) | (1 << 16));

    if (get_voltage_(gpu_, &voltage) != 0)
        return -1;

    int millivolts = (int)(voltage.rails[0].current_voltage_uv / MICROVOLTS_PER_MILLIVOLT);

    if (millivolts <= 0 || millivolts >= MAX_PLAUSIBLE_MILLIVOLTS)
        return -1;

    return millivolts;
}
