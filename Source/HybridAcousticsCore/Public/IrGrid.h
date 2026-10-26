#pragma once
#include <cstddef>
#include <vector>
#include "Export.h"

namespace fdtd {

    // Ergebnis eines Bakes: eine Impulsantwort pro Empfänger auf einem regelmäßigen Gitter.
    // Koordinaten in Metern, Ursprung in einer Raumecke, Z = Höhe.
    struct FDTD_API IrGrid {
        int nx = 0, ny = 0, nz = 0;   // Anzahl Empfänger je Achse
        float spacing = 0.f;          // Abstand der Empfänger (m)
        int length = 0;               // Samples pro IR
        int sampleRate = 0;           // Hz
        std::vector<float> data;      // [receiverIndex * length + n]

        int numReceivers() const { return nx * ny * nz; }
        int index(int ix, int iy, int iz) const { return (iz * ny + iy) * nx + ix; }
        const float* ir(int idx) const { return data.data() + (std::size_t)idx * length; }

        // Mittelpunkt des Empfängers idx (m)
        void position(int idx, float& x, float& y, float& z) const;

        // Index des nächsten Empfängers; Positionen außerhalb werden auf den Rand geklemmt -> nie leer
        int nearest(float x, float y, float z) const;

    };

} // namespace fdtd