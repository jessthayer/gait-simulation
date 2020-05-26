
%% apply appropriate subject parameters to segment lengths
subjID = 'Subject_2'; %give subject ID here
load(['Param_',subjID],'param');

%apply subject parameters to test model
    %open plant model
    load('ds_pose_test_mat')
    open_system('ds_pose_test');
    sys = 'ds_pose_test';

    %write subject parameters
    for k = 1:22
        sdo.setValueInModel(sys,param.var(k),param.val(k))
    end
    
    %close model
    save_system(sys)
    sim(sys)
    
%% write positions and velocities
    sdo.setValueInModel(sys,'p_stance_ankle',ic.p(1));
    sdo.setValueInModel(sys,'p_stance_knee',ic.p(2));
    sdo.setValueInModel(sys,'p_stance_hip',ic.p(3));
    sdo.setValueInModel(sys,'p_swing_ankle',ic.p(4));
    sdo.setValueInModel(sys,'p_swing_knee',ic.p(5));
    sdo.setValueInModel(sys,'p_swing_hip',ic.p(6));
    sdo.setValueInModel(sys,'w_stance_ankle',ic.w(1));
    sdo.setValueInModel(sys,'w_stance_knee',ic.w(2));
    sdo.setValueInModel(sys,'w_stance_hip',ic.w(3));
    sdo.setValueInModel(sys,'w_swing_ankle',ic.w(4));
    sdo.setValueInModel(sys,'w_swing_knee',ic.w(5));
    sdo.setValueInModel(sys,'w_swing_hip',ic.w(6));
    
    sim(sys)
    
%% save initial conditions set for subject ds sims

save(['IC_ds_',subjID],'ic')