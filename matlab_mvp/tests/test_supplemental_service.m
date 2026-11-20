function tests=test_supplemental_service
tests=functiontests(localfunctions);
end
function testSupplementNeedsRecentUnfinishedSpatialProgress(t)
cfg=uavmvp.config();state=struct('phase','LOCALIZE','pool_ids',1);
node=struct('last_contact_s',10);
verifyFalse(t,uavmvp.supplemental_service_needed(state,node,11,cfg));
state.pool_ids=[1,2];verifyTrue(t,uavmvp.supplemental_service_needed(state,node,11,cfg));
verifyFalse(t,uavmvp.supplemental_service_needed(state,node,21,cfg));
state.phase='DONE';verifyFalse(t,uavmvp.supplemental_service_needed(state,node,11,cfg));
state.phase='FAILED';verifyFalse(t,uavmvp.supplemental_service_needed(state,node,11,cfg));
end
function testLastDiscoveryCanSeekSecondPositionWithoutContactOracle(t)
cfg=uavmvp.config();states=repmat(struct('phase','DONE','pool_ids',1:5),1,2);
states(2).phase='LOCALIZE';states(2).pool_ids=1;
nodes=repmat(struct('discovered',true,'last_contact_s',10),1,2);
% Fixed-route guard caused starvation when carried into the online planner.
verifyFalse(t,uavmvp.supplemental_service_needed(states,nodes,11,cfg));
verifyTrue(t,uavmvp.supplemental_service_needed(states,nodes,11,cfg,'online'));
% Old contact does not prove either future failure or future success: permit
% candidate evaluation, whose model and actual execution check service.
verifyTrue(t,uavmvp.supplemental_service_needed(states,nodes,100,cfg,'online'));
nodes(1).discovered=false;
verifyFalse(t,uavmvp.supplemental_service_needed(states,nodes,11,cfg,'online'));
nodes(1).discovered=true;states(2).phase='DONE';
verifyFalse(t,uavmvp.supplemental_service_needed(states,nodes,11,cfg,'online'));
end
