#pragma once
#include <cstdint>

// Reads sensors that NVML does not expose on GeForce cards by going through
// NVIDIA's own NVAPI library (libnvidia-api.so.1). No elevated privileges and
// no direct BAR0 access are needed: the driver performs the read for us.
//
// Blackwell reports the hotspot through a different path and is deliberately not
// handled - hotspot() simply reports nothing there rather than a wrong number.
class NvApiSensors {
    public:
        ~NvApiSensors();

        // pci_bus_id is the usual "0000:01:00.0" form.
        bool init(const char* pci_bus_id);
        bool available() const { return available_; }

        // Integer degrees celsius, truncated, or -1 when unavailable.
        int hotspot();
        int vram();

        // Millivolts, or -1 when unavailable.
        int voltage();

    private:
        struct Thermals {
            uint32_t version;
            int32_t mask;
            int32_t values[40];
        };

        struct VoltageRail {
            uint32_t rail_id;
            uint32_t current_voltage_uv;
            uint8_t reserved[32];
        };

        struct Voltage {
            uint32_t version;
            uint8_t reserved[32];
            VoltageRail rails[1];
        };

        static const int HOTSPOT_INDEX = 9;

        // Verified on GA107 (Ampere, GDDR6) by comparing a shader-bound load with
        // a memory-bound one at matched core temperature: every die sensor stayed
        // within 0.16 C of the core across both, while this one rose 1.24 C once
        // the memory bus was saturated. It also reports whole degrees only, unlike
        // the 1/256 C die taps. Other parts may well place memory elsewhere.
        static const int VRAM_INDEX = 16;

        // Ada (AD102, GDDR6X) accepts an 8-bit sensor mask and fills indices 8..15,
        // so the verified GA107 index falls off the end and the readout is empty.
        // Its memory die sits at the last filled slot, which is base + 7, where base
        // is the first index NVAPI writes for the accepted mask. Measured on an
        // RTX 4090 with driver 620.70.06: base 8, index 15 = 50.00 C, matching LACT.
        static const int VRAM_INDEX_OFFSET = 7;

        // Ada (AD102, GDDR6X) accepts an 8-bit sensor mask and fills indices 8..15,
        // so the verified GA107 index falls off the end and the readout is empty.
        // Its memory die sits at the last filled slot, which is base + 7, where base
        // is the first index NVAPI writes for the accepted mask. Measured on an
        // RTX 4090 with driver 620.70.06: base 8, index 15 = 50.00 C, matching LACT.
        static const int VRAM_INDEX_OFFSET = 7;

        void* library_ = nullptr;
        void* (*query_interface_)(uint32_t) = nullptr;
        int32_t (*get_thermals_)(void*, Thermals*) = nullptr;
        int32_t (*get_voltage_)(void*, Voltage*) = nullptr;
        void* gpu_ = nullptr;
        int32_t mask_ = 0;
        int base_ = -1;
        bool available_ = false;

        bool find_gpu(unsigned bus);
        bool calculate_mask();
        int read_sensor(int index);
        int read_thermals(int* out);
        int detect_base();
        int memory_index();
};
