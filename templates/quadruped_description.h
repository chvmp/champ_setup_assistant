#ifndef QUADRUPED_DESCRIPTION_H
#define QUADRUPED_DESCRIPTION_H

#include <quadruped_base/quadruped_leg.h>
#include <quadruped_base/quadruped_base.h>

champ::Joint        lf_hip({{ left_front["hip"][0] }}, {{ left_front["hip"][1] }}, {{ left_front["hip"][2] }}, {{ left_front["hip"][3] }}, {{ left_front["hip"][4] }}, {{ left_front["hip"][5] }});
champ::Joint  lf_upper_leg({{ left_front["upper_leg"][0] }}, {{ left_front["upper_leg"][1] }}, {{ left_front["upper_leg"][2] }}, {{ left_front["upper_leg"][3] }}, {{ left_front["upper_leg"][4] }}, {{ left_front["upper_leg"][5] }});
champ::Joint  lf_lower_leg({{ left_front["lower_leg"][0] }}, {{ left_front["lower_leg"][1] }}, {{ left_front["lower_leg"][2] }}, {{ left_front["lower_leg"][3] }}, {{ left_front["lower_leg"][4] }}, {{ left_front["lower_leg"][5] }});
champ::Joint       lf_foot({{ left_front["foot"][0] }}, {{ left_front["foot"][1] }}, {{ left_front["foot"][2] }}, {{ left_front["foot"][3] }}, {{ left_front["foot"][4] }}, {{ left_front["foot"][5] }});

champ::Joint        rf_hip({{ right_front["hip"][0] }}, {{ right_front["hip"][1] }}, {{ right_front["hip"][2] }}, {{ right_front["hip"][3] }}, {{ right_front["hip"][4] }}, {{ right_front["hip"][5] }});
champ::Joint  rf_upper_leg({{ right_front["upper_leg"][0] }}, {{ right_front["upper_leg"][1] }}, {{ right_front["upper_leg"][2] }}, {{ right_front["upper_leg"][3] }}, {{ right_front["upper_leg"][4] }}, {{ right_front["upper_leg"][5] }});
champ::Joint  rf_lower_leg({{ right_front["lower_leg"][0] }}, {{ right_front["lower_leg"][1] }}, {{ right_front["lower_leg"][2] }}, {{ right_front["lower_leg"][3] }}, {{ right_front["lower_leg"][4] }}, {{ right_front["lower_leg"][5] }});
champ::Joint       rf_foot({{ right_front["foot"][0] }}, {{ right_front["foot"][1] }}, {{ right_front["foot"][2] }}, {{ right_front["foot"][3] }}, {{ right_front["foot"][4] }}, {{ right_front["foot"][5] }});

champ::Joint        lh_hip({{ left_hind["hip"][0] }}, {{ left_hind["hip"][1] }}, {{ left_hind["hip"][2] }}, {{ left_hind["hip"][3] }}, {{ left_hind["hip"][4] }}, {{ left_hind["hip"][5] }});
champ::Joint  lh_upper_leg({{ left_hind["upper_leg"][0] }}, {{ left_hind["upper_leg"][1] }}, {{ left_hind["upper_leg"][2] }}, {{ left_hind["upper_leg"][3] }}, {{ left_hind["upper_leg"][4] }}, {{ left_hind["upper_leg"][5] }});
champ::Joint  lh_lower_leg({{ left_hind["lower_leg"][0] }}, {{ left_hind["lower_leg"][1] }}, {{ left_hind["lower_leg"][2] }}, {{ left_hind["lower_leg"][3] }}, {{ left_hind["lower_leg"][4] }}, {{ left_hind["lower_leg"][5] }});
champ::Joint       lh_foot({{ left_hind["foot"][0] }}, {{ left_hind["foot"][1] }}, {{ left_hind["foot"][2] }}, {{ left_hind["foot"][3] }}, {{ left_hind["foot"][4] }}, {{ left_hind["foot"][5] }});

champ::Joint        rh_hip({{ right_hind["hip"][0] }}, {{ right_hind["hip"][1] }}, {{ right_hind["hip"][2] }}, {{ right_hind["hip"][3] }}, {{ right_hind["hip"][4] }}, {{ right_hind["hip"][5] }});
champ::Joint  rh_upper_leg({{ right_hind["upper_leg"][0] }}, {{ right_hind["upper_leg"][1] }}, {{ right_hind["upper_leg"][2] }}, {{ right_hind["upper_leg"][3] }}, {{ right_hind["upper_leg"][4] }}, {{ right_hind["upper_leg"][5] }});
champ::Joint  rh_lower_leg({{ right_hind["lower_leg"][0] }}, {{ right_hind["lower_leg"][1] }}, {{ right_hind["lower_leg"][2] }}, {{ right_hind["lower_leg"][3] }}, {{ right_hind["lower_leg"][4] }}, {{ right_hind["lower_leg"][5] }});
champ::Joint       rh_foot({{ right_hind["foot"][0] }}, {{ right_hind["foot"][1] }}, {{ right_hind["foot"][2] }}, {{ right_hind["foot"][3] }}, {{ right_hind["foot"][4] }}, {{ right_hind["foot"][5] }});

champ::QuadrupedLeg lf_leg(lf_hip, lf_upper_leg, lf_lower_leg, lf_foot);
champ::QuadrupedLeg rf_leg(rf_hip, rf_upper_leg, rf_lower_leg, rf_foot);;
champ::QuadrupedLeg lh_leg(lh_hip, lh_upper_leg, lh_lower_leg, lh_foot);
champ::QuadrupedLeg rh_leg(rh_hip, rh_upper_leg, rh_lower_leg, rh_foot);

#endif