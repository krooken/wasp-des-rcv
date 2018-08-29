classdef ref_controller_mode_dt < Simulink.IntEnumType
    
    enumeration
        nominal(1)
        safe_maneuver(2)
        emergency_maneuver(3)
        ref_controller_mode_unknown(0)
    end
    
end

