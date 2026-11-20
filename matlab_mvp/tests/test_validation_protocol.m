function tests=test_validation_protocol
tests=functiontests(localfunctions);
end
function testHeldOutPlanIsBalancedAndDisjointFromDevelopment(t)
dev=validation_cases('development');final=validation_cases('independent');
verifyNumElements(t,dev,42);verifyNumElements(t,final,280);
verifyEqual(t,numel(unique({final.id})),280);
verifyEmpty(t,intersect([dev.seed],[final.seed]));
for seed=31001:31020
    for count=[5,10]
        rows=final([final.seed]==seed & [final.count]==count);
        verifyNumElements(t,rows,7);verifyEqual(t,numel(unique({rows.method})),7);
        profiles=[rows.profile];
        verifyEqual(t,unique({profiles.prediction_model}),{'conditional_reference'});
        verifyTrue(t,all([profiles.prepared_transit]));
    end
end
end
function testAblationPlansChangeOnlyScoringFactor(t)
cases=validation_cases('development');cases=cases(1:7);base=cases(1).profile;
for j=5:7
    verifyEqual(t,cases(j).policy,cases(1).policy);
    verifyEqual(t,rmfield(cases(j).profile,'scoring_ablation'),rmfield(base,'scoring_ablation'));
end
verifyEqual(t,{cases(5:7).method},{'A_no_stage','A_no_sharing','A_local_cost'});
profiles=[cases.profile];
verifyEqual(t,{profiles(5:7).scoring_ablation},{'no_stage','no_sharing','local_cost'});
end
function testRegressionRetainsOriginalProfileAndFourPhysicalCases(t)
cases=validation_cases('regression');verifyNumElements(t,cases,4);
verifyEqual(t,[cases.count],[5,10,10,10]);verifyEqual(t,[cases.seed],[81,81,82,83]);
profiles=[cases.profile];verifyEqual(t,unique({profiles.prediction_model}),{'arrival'});
verifyEqual(t,unique({profiles.scoring_ablation}),{'none'});
verifyError(t,@()validation_cases('unplanned'),'uavmvp:validationStage');
end
function testCurrentScenarioKeepsConfirmedNumericalConstraints(t)
cfg=uavmvp.config();verifyEqual(t,cfg.scene.application_confirmed_date,'2026-11-13');
verifyTrue(t,contains(cfg.scene.application,'military'));verifyTrue(t,contains(cfg.scene.application_scope,'Above-ground'));
verifyEqual(t,[cfg.scene.width_m,cfg.scene.height_m,cfg.scene.flight_height_m],[120,80,15]);
verifyEqual(t,cfg.motion_and_budget.time_limit_s,600);
verifyEqual(t,cfg.localization.offline_position_error_limit_m,20);
verifyEqual(t,cfg.motion_and_budget.return_position_tolerance_m,3);
verifyEqual(t,cfg.localization.fit_position_count,4);verifyEqual(t,cfg.localization.heldout_position_count,1);
verifyFalse(t,cfg.navigation_candidate.unknown_terminal_rtt_updates_navigation);
end
