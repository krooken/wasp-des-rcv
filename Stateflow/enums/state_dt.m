classdef state_dt < Simulink.IntEnumType
    
    enumeration
        state_waiting_for_goal(0)
        state_wait_for_transforms(1)
        state_call_for_plan(2)
        state_failed(3)
        state_parking_to_parking(4)
        state_parking_to_road(5)
        state_road_to_parking(6)
        state_road_to_road(7)
        state_from_parking(8)
        state_transition_to_spp(9)
        state_wait_to_leave_tp1(10)
        state_spp(11)
        state_close_to_tp2(12)
        state_transition_to_upp2(13)
        state_wait_to_leave_tp2(14)
        state_go_to_goal(15)
        state_goal(16)
        state_ssmp(17)
        state_safe_stop(18)
        state_aeb(19)
        state_emergency_stop(20)
        state_unknown(21)
    end
    
end