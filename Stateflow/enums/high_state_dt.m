classdef high_state_dt < Simulink.IntEnumType
    
    enumeration
        state_init(0)
        state_driving(1)
        state_failure(2)
        state_high_unknown(3)
    end
    
end