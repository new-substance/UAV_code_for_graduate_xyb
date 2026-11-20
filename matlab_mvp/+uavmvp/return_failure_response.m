function action=return_failure_response(step,from_id)
%RETURN_FAILURE_RESPONSE Do not label an unobserved destination as failed.
action=struct('block_target',false,'repair_departure',false,'stop',false,'reason','NO_FAILURE');
if step.accepted,return;end
if strcmp(step.reason,'REFERENCE_BUDGET')
    action.stop=true;action.reason='BUDGET_EXHAUSTED';
elseif step.arrived && ~isempty(step.attempts)
    action.block_target=true;action.reason='TARGET_OBSERVATIONS_FAILED';
elseif from_id~=step.id
    action.repair_departure=true;action.reason='TARGET_NOT_OBSERVED';
else
    action.stop=true;action.reason='NO_DISTINCT_DEPARTURE_REPAIR';
end
end
