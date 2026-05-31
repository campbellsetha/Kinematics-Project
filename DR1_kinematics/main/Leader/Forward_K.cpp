#include "Forward_K.hpp"
 
// ------------------------------------------------------------
// ForwardK implementation is header-only for this class —
// all methods are defined inline in Forward_K.hpp because
// they are small and the ESP32 toolchain benefits from inlining
// the tight compute loop.
//
// This file exists to satisfy the build system and to hold
// any future non-inline ForwardK methods if complexity grows.
// ------------------------------------------------------------
 
// ------------------------------------------------------------
// Usage example — integrate into your leader control loop:
//
//   Frame frame;
//   initChain(frame);
//   ForwardK fk(frame, calibration_offsets);
//
//   // Each control cycle, when new ticks arrive over ESPNow:
//   uint16_t ticks[5] = { ... };  // from leader servo reads
//   std::array<double, 3> pos = fk.compute(ticks);
//
//   // pos[0] = x metres from base
//   // pos[1] = y metres from base
//   // pos[2] = z metres from base
//
//   // Transmit pos to follower for InverseK input
// ------------------------------------------------------------
 
