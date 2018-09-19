
%[pos,vel,acc,jerk,alf]=trajgen(totdur,nrofmeas,dist,velmax,accmax)
%
% input: totdur,velmax,accmax,nrofmeas,dist
% output: pos,vel,acc,jerk,alf


function [pos,vel,acc,jerk,alf] = trajgen(totdur,nrofmeas,dist,velmax,accmax)
pos=zeros(1,nrofmeas);
vel=pos;
acc=pos;
talf=pos;
jerk=pos;
alf=pos;

absdist = abs(dist);
mindist = 2 * velmax * velmax /accmax;
if absdist >= mindist
       t1 = ceil(velmax/2/accmax);
       t2 = ceil(absdist/(2*accmax*t1)) - 4 * t1;
else
       t1 = ceil(sqrt(absdist/(8*accmax)));
       t2 = 0;
end
tmin = 8*t1 + t2;
if totdur < tmin
       fprintf('Minimum time duration in samples: %g\n',tmin);
       totdur = tmin;
elseif totdur > tmin
       t1 = round((totdur - ceil(sqrt(totdur*totdur - 8.0*absdist/accmax)))/8);
       if t1 <= 0
            t1 = 1;
       end
       t2 = totdur - 8*t1;
end
%  Calculation of ALFA
alfa = dist/(2.0*t1*t1*t1*(4.0*t1+t2));

fprintf('alfa,t1,t2 : %g    %g    %g\n ',alfa,t1,t2);
for i=2:t1+1
        alf(i)  = alfa;
end
for i=t1+2:3*t1+1
        alf(i) = -alfa;
end
for i=3*t1+2:4*t1+1
        alf(i) = alfa;
end
t3 = t2 + 4*t1;
for i=t3+2:t3+t1+1
        alf(i) = -alfa;
end
for i=t3+t1+2:t3+3*t1+1
        alf(i) = alfa;
end
for i=t3+3*t1+2:t3+4*t1+1
        alf(i) = -alfa;
end
for i=2:totdur
     jerk(i)=jerk(i-1) + alf(i);
     acc(i) =acc(i-1)  + jerk(i) - alf(i)/2;
     vel(i) =vel(i-1)  + acc(i)  - jerk(i)/2 + alf(i)/6;
     pos(i) =pos(i-1)  + vel(i)  - acc(i)/2  + jerk(i)/6 - alf(i)/24;
end
for i=totdur:nrofmeas
     pos(i) = dist;
end
