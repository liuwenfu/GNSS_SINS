#pragma once
#include "util.h"

void extern IMUupdate(Vect *wm, Vect *vm, int16u nSamples, Vect *phim, Vect *dvbm);
void extern IMUupdate1(Vect *wm, Vect *vm, int16u nSamples, Vect *phim, Vect *dvbm, int16u coneoptimal);