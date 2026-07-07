#include "Noise.h"
#include "AtmosphereSimulation.h"
#include <Arduino.h>

// Simple pseudo‑noise based on trigonometric functions.
// Returns a density value in the range [0, 1].
float getDensity(int x, int y, int z, float time) {
    // Normalized coordinates in [0,1]
    float nx = (float)x / (float)AtmosphereSimulation::X_SIZE;
    float ny = (float)y / (float)AtmosphereSimulation::Y_SIZE;
    float nz = (float)z / (float)AtmosphereSimulation::Z_SIZE;

    // Base frequencies for different octaves
    const float freq1 = 4.0f;   // coarse structure
    const float freq2 = 12.0f;  // finer detail

    // Octave 1
    float n1 = sinf(nx * freq1 * 2.0f * PI + time) *
               cosf(ny * freq1 * 2.0f * PI + time * 0.7f) *
               sinf(nz * freq1 * 2.0f * PI + time * 0.4f);

    // Octave 2 (higher frequency, lower amplitude)
    float n2 = sinf(nx * freq2 * 2.0f * PI + time * 1.3f) *
               cosf(ny * freq2 * 2.0f * PI + time * 0.9f) *
               sinf(nz * freq2 * 2.0f * PI + time * 0.6f);

    // Combine octaves (simple weighted sum)
    float noise = 0.7f * n1 + 0.3f * n2;

    // Map from [-1,1] to [0,1]
    // Scale to [0,1] then dampen to lower overall density (~0.4 max)
    return ((noise + 1.0f) * 0.5f) * 0.4f;
}
