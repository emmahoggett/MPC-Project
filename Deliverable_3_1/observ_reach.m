function observ_reach(sys, name)
%OBSERV_REACH Check if the system is observable and reachable
%   - sys : system -

    [~] = reachability(sys,name);
    [~] = observability(sys,name);
end

function [ranko] = observability(sys, name)
%OBSERVABILTY Check if the system is observable or not
     Mo = obsv(sys.A,sys.C);
     'Observability check : '
     if (rank(Mo) == size(sys.A,1))
         fprintf ( 'The system is observable on %s\n', name)
         ranko = true;
     else
         fprintf ( 'The system is unobservable on %s\n', name)
         ranko = false;
     end
     
end

function [rankr] = reachability(sys, name)
%OBSERVABILTY Check if the system is reachable or not
     Mr = ctrb(sys.A,sys.B);
     'Reachability check : '
     if (rank(Mr) == size(sys.A,1))
         fprintf ('The system is reachable on %s', name)
         rankr = true;
     else
         fprintf ('The system is unreachable on %s', name)
         rankr = false;
     end
     
end

