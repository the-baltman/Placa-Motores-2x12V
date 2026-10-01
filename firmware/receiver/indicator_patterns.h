// indicator_patterns.h - WS2812 colour/pattern table (Indicadores-y-failsafe.md sec. 2) as a
// pure function of state and time. Full-scale colours; brightness is applied by indicators.cpp.
#pragma once
#include <stdint.h>

#include "supervisor.h"

struct Rgb {
  uint8_t r, g, b;
};

constexpr Rgb RGB_OFF = {0, 0, 0};

// t_ms = time since the state was entered. fault_mask: bit0 = channel 1, bit1 = channel 2.
inline Rgb indicator_rgb(SysState s, uint8_t fault_mask, uint32_t t_ms) {
  switch (s) {
    case SysState::INICIANDO:  // blue, 1 Hz 500/500
      return (t_ms % 1000) < 500 ? Rgb{0, 0, 255} : RGB_OFF;
    case SysState::SIN_ENLACE: {  // yellow, breathing 1 Hz (triangle, floor 5 %)
      const uint32_t ph = t_ms % 1000;
      const uint32_t tri = ph < 500 ? ph : 1000 - ph;  // 0..500
      const uint32_t k = 13 + (tri * 242) / 500;       // 13..255
      return Rgb{(uint8_t)((255 * k) / 255), (uint8_t)((160 * k) / 255), 0};
    }
    case SysState::ENLACE_OK:  // green, solid
      return Rgb{0, 255, 0};
    case SysState::FAILSAFE_ENLACE:  // orange, 4 Hz
      return (t_ms % 250) < 125 ? Rgb{255, 80, 0} : RGB_OFF;
    case SysState::ESTOP:  // red, solid
      return Rgb{255, 0, 0};
    case SysState::FALLA_CANAL: {  // red, 2 Hz flashes: 1 = channel 1, 2 = channel 2, 3 = both; then 1 s pause
      const uint32_t n = fault_mask == 1 ? 1 : (fault_mask == 2 ? 2 : 3);
      const uint32_t period = n * 250 + 1000;
      const uint32_t ph = t_ms % period;
      return (ph < n * 250 && (ph % 250) < 125) ? Rgb{255, 0, 0} : RGB_OFF;
    }
    case SysState::FALLA_SISTEMA:  // red, fast 8 Hz (distinct from the 2 Hz channel fault)
      return (t_ms % 125) < 62 ? Rgb{255, 0, 0} : RGB_OFF;
    case SysState::SOBRETEMP:  // red / orange alternating, 1 Hz
      return (t_ms % 1000) < 500 ? Rgb{255, 0, 0} : Rgb{255, 80, 0};
    case SysState::BATERIA_BAJA: {  // magenta, double flash every 2 s
      const uint32_t ph = t_ms % 2000;
      return (ph < 100 || (ph >= 200 && ph < 300)) ? Rgb{255, 0, 255} : RGB_OFF;
    }
    case SysState::WDT_DISPARADO:  // white, short flash every 1 s
      return (t_ms % 1000) < 100 ? Rgb{255, 255, 255} : RGB_OFF;
  }
  return RGB_OFF;
}
