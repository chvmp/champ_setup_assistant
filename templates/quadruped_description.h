#ifndef QUADRUPED_DESCRIPTION_H
#define QUADRUPED_DESCRIPTION_H

#include <quadruped_base/quadruped_base.h>

namespace champ
{
    namespace URDF
    {
        void loadFromHeader(champ::QuadrupedBase &base)
        {
      base.lf.hip.setOrigin({{ left_front["hip"][0] }}, {{ left_front["hip"][1] }}, {{ left_front["hip"][2] }}, {{ left_front["hip"][3] }}, {{ left_front["hip"][4] }}, {{ left_front["hip"][5] }});
base.lf.upper_leg.setOrigin({{ left_front["upper_leg"][0] }}, {{ left_front["upper_leg"][1] }}, {{ left_front["upper_leg"][2] }}, {{ left_front["upper_leg"][3] }}, {{ left_front["upper_leg"][4] }}, {{ left_front["upper_leg"][5] }});
base.lf.lower_leg.setOrigin({{ left_front["lower_leg"][0] }}, {{ left_front["lower_leg"][1] }}, {{ left_front["lower_leg"][2] }}, {{ left_front["lower_leg"][3] }}, {{ left_front["lower_leg"][4] }}, {{ left_front["lower_leg"][5] }});
     base.lf.foot.setOrigin({{ left_front["foot"][0] }}, {{ left_front["foot"][1] }}, {{ left_front["foot"][2] }}, {{ left_front["foot"][3] }}, {{ left_front["foot"][4] }}, {{ left_front["foot"][5] }});

      base.rf.hip.setOrigin({{ right_front["hip"][0] }}, {{ right_front["hip"][1] }}, {{ right_front["hip"][2] }}, {{ right_front["hip"][3] }}, {{ right_front["hip"][4] }}, {{ right_front["hip"][5] }});
base.rf.upper_leg.setOrigin({{ right_front["upper_leg"][0] }}, {{ right_front["upper_leg"][1] }}, {{ right_front["upper_leg"][2] }}, {{ right_front["upper_leg"][3] }}, {{ right_front["upper_leg"][4] }}, {{ right_front["upper_leg"][5] }});
base.rf.lower_leg.setOrigin({{ right_front["lower_leg"][0] }}, {{ right_front["lower_leg"][1] }}, {{ right_front["lower_leg"][2] }}, {{ right_front["lower_leg"][3] }}, {{ right_front["lower_leg"][4] }}, {{ right_front["lower_leg"][5] }});
     base.rf.foot.setOrigin({{ right_front["foot"][0] }}, {{ right_front["foot"][1] }}, {{ right_front["foot"][2] }}, {{ right_front["foot"][3] }}, {{ right_front["foot"][4] }}, {{ right_front["foot"][5] }});

      base.lh.hip.setOrigin({{ left_hind["hip"][0] }}, {{ left_hind["hip"][1] }}, {{ left_hind["hip"][2] }}, {{ left_hind["hip"][3] }}, {{ left_hind["hip"][4] }}, {{ left_hind["hip"][5] }});
base.lh.upper_leg.setOrigin({{ left_hind["upper_leg"][0] }}, {{ left_hind["upper_leg"][1] }}, {{ left_hind["upper_leg"][2] }}, {{ left_hind["upper_leg"][3] }}, {{ left_hind["upper_leg"][4] }}, {{ left_hind["upper_leg"][5] }});
base.lh.lower_leg.setOrigin({{ left_hind["lower_leg"][0] }}, {{ left_hind["lower_leg"][1] }}, {{ left_hind["lower_leg"][2] }}, {{ left_hind["lower_leg"][3] }}, {{ left_hind["lower_leg"][4] }}, {{ left_hind["lower_leg"][5] }});
     base.lh.foot.setOrigin({{ left_hind["foot"][0] }}, {{ left_hind["foot"][1] }}, {{ left_hind["foot"][2] }}, {{ left_hind["foot"][3] }}, {{ left_hind["foot"][4] }}, {{ left_hind["foot"][5] }});

      base.rh.hip.setOrigin({{ right_hind["hip"][0] }}, {{ right_hind["hip"][1] }}, {{ right_hind["hip"][2] }}, {{ right_hind["hip"][3] }}, {{ right_hind["hip"][4] }}, {{ right_hind["hip"][5] }});
base.rh.upper_leg.setOrigin({{ right_hind["upper_leg"][0] }}, {{ right_hind["upper_leg"][1] }}, {{ right_hind["upper_leg"][2] }}, {{ right_hind["upper_leg"][3] }}, {{ right_hind["upper_leg"][4] }}, {{ right_hind["upper_leg"][5] }});
base.rh.lower_leg.setOrigin({{ right_hind["lower_leg"][0] }}, {{ right_hind["lower_leg"][1] }}, {{ right_hind["lower_leg"][2] }}, {{ right_hind["lower_leg"][3] }}, {{ right_hind["lower_leg"][4] }}, {{ right_hind["lower_leg"][5] }});
     base.rh.foot.setOrigin({{ right_hind["foot"][0] }}, {{ right_hind["foot"][1] }}, {{ right_hind["foot"][2] }}, {{ right_hind["foot"][3] }}, {{ right_hind["foot"][4] }}, {{ right_hind["foot"][5] }});
        }
    }
}
#endif