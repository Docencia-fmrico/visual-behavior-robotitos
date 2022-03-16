// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cstdlib>

#include "visual_behavior/PID.h"
#include "visual_behavior/clamp.h"

namespace visual_behavior
{

PID::PID(double min_ref, double max_ref, double min_output, double max_output) //constructor para definir los limites del PID
{
  min_ref_ = min_ref;
  max_ref_ = max_ref;
  min_output_ = min_output;
  max_output_ = max_output;
  prev_error_ = int_error_ = 0.0;

  KP_ = 0.41;
  KI_ = 0.06;
  KD_ = 0.53;
}

void
PID::set_pid(double n_KP, double n_KI, double n_KD)
{
  KP_ = n_KP;
  KI_ = n_KI;
  KD_ = n_KD;
}

double
PID::get_output(double new_reference) //con esto nos da el resultado ya pasado con el PID
{
  double ref = new_reference;
  double output = 0.0;

  // Proportional Error
  double direction = 0.0;
  if (ref != 0.0) {
    direction = ref / fabs(ref);
  }

  if (fabs(ref) < min_ref_) {
    output = 0.0;
  } else if (fabs(ref) > max_ref_) {
    output = direction * max_output_;
  } else {
    output = direction * min_output_ + ref * (max_output_ - min_output_);
  }

  // Integral Error
  int_error_ = (int_error_ + output) * 2.0 / 3.0;

  // Derivative Error
  double deriv_error = output - prev_error_;
  prev_error_ = output;

  output = KP_ * output + KI_ * int_error_ + KD_ * deriv_error;

  return clamp(output, -max_output_, max_output_);
}

}  // namespace visual_behavior
