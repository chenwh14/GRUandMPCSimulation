function [posSeq,velSeq] = TrapezoidalPathGen(s,v,ts,vmax,amax,pathSizeMax)
%用梯形法由最大速度、最大加速度规划路径
%   s:相对位移
%   v:当前速度
%   ts:采样周期
%   vmax:最大速度
%   amax:最大加速度

if v<0
    [posSeq_,velSeq_]=TrapezoidalPathGen(-s,-v,ts,vmax,amax,pathSizeMax);
    posSeq=-posSeq_;
    velSeq=-velSeq_;
else
    %%
    %只需考虑v>0的情况
    if v<=vmax%如果v小于vmax，正常情况
        t_0_2_vm=vmax/amax;%从0加速至vmax的时间
        x_0_2_vm=1/2*vmax*t_0_2_vm;%从0加速至vmax时走过的路程
        t_v_2_vm=(vmax-v)/amax;%从v加速至vmax的时间
        x_v_2_vm=1/2*(vmax+v)*t_v_2_vm;%从v加速至vmax时走过的路程
        t_0_2_v=v/amax;%从0加速至v的时间
        x_0_2_v=1/2*v*t_0_2_v;
        
        
        %如果s足够长，则路径是加速到最大，然后匀速，然后减速到0
        if s>x_v_2_vm+x_0_2_vm
            t_pacc=t_v_2_vm;%正向加速
            t_pconst=(s-x_v_2_vm-x_0_2_vm)/vmax;%正向匀速
            t_pdeacc=t_0_2_vm;%正向减速
            t_nacc=0;%反向加速
            t_nconst=0;%反向匀速
            t_ndeacc=0;%反向减速
            
            if(round((t_pacc+t_pconst+t_pdeacc+t_nacc+t_nconst+t_ndeacc)/ts)<=pathSizeMax)
                posSeq=zeros(round((t_pacc+t_pconst+t_pdeacc+t_nacc+t_nconst+t_ndeacc)/ts),1);
                velSeq=zeros(round((t_pacc+t_pconst+t_pdeacc+t_nacc+t_nconst+t_ndeacc)/ts),1);
            else
                posSeq=zeros(pathSizeMax,1);
                velSeq=zeros(pathSizeMax,1);
            end
            for i=1:1:size(posSeq,1)
                t=i*ts;
                if t<=t_pacc
                    posSeq(i)=v*t+1/2*amax*t*t;
                    velSeq(i)=v+amax*t;
                else if t<t_pacc+t_pconst
                        t=t-t_pacc;
                        posSeq(i)=x_v_2_vm+vmax*t;
                        velSeq(i)=vmax;
                    else
                        t=t-t_pacc-t_pconst;
                        posSeq(i)=x_v_2_vm+vmax*t_pconst+vmax*t-1/2*amax*t*t;
                        velSeq(i)=vmax-amax*t;
                    end
                end
            end
        end
        %如果s稍短，则先加速然后减速到0
        if s<=x_v_2_vm+x_0_2_vm && s>x_0_2_v
            t_pacc=sqrt((s+x_0_2_v)/amax)-t_0_2_v;%正向加速
            t_pconst=0;%正向匀速
            t_pdeacc=sqrt((s+x_0_2_v)/amax);%正向减速
            t_nacc=0;%反向加速
            t_nconst=0;%反向匀速
            t_ndeacc=0;%反向减速
            
            if(round((t_pacc+t_pconst+t_pdeacc+t_nacc+t_nconst+t_ndeacc)/ts)<=pathSizeMax)
                posSeq=zeros(round((t_pacc+t_pconst+t_pdeacc+t_nacc+t_nconst+t_ndeacc)/ts),1);
                velSeq=zeros(round((t_pacc+t_pconst+t_pdeacc+t_nacc+t_nconst+t_ndeacc)/ts),1);
            else
                posSeq=zeros(pathSizeMax,1);
                velSeq=zeros(pathSizeMax,1);
            end
            for i=1:1:size(posSeq,1)
                t=i*ts;
                if t<=t_pacc
                    posSeq(i)=v*t+1/2*amax*t*t;
                    velSeq(i)=v+amax*t;
                else
                    t=t-t_pacc;
                    posSeq(i)=v*t_pacc+1/2*amax*t_pacc*t_pacc+(v+t_pacc*amax)*t-1/2*amax*t*t;
                    velSeq(i)=v+amax*t_pacc-amax*t;
                end
            end
        end
        %如果s很短甚至为负，则需要立即减速，减速到零后反向加速，然后减速到0
        if s>x_0_2_v-2*x_0_2_vm && s<=x_0_2_v
            t_pacc=0;%正向加速
            t_pconst=0;%正向匀速
            t_pdeacc=t_0_2_v;%正向减速
            t_nacc=sqrt((x_0_2_v-s)/amax);%反向加速
            t_nconst=0;%反向匀速
            t_ndeacc=sqrt((x_0_2_v-s)/amax);%反向减速
            
            if(round((t_pacc+t_pconst+t_pdeacc+t_nacc+t_nconst+t_ndeacc)/ts)<=pathSizeMax)
                posSeq=zeros(round((t_pacc+t_pconst+t_pdeacc+t_nacc+t_nconst+t_ndeacc)/ts),1);
                velSeq=zeros(round((t_pacc+t_pconst+t_pdeacc+t_nacc+t_nconst+t_ndeacc)/ts),1);
            else
                posSeq=zeros(pathSizeMax,1);
                velSeq=zeros(pathSizeMax,1);
            end
            for i=1:1:size(posSeq,1)
                t=i*ts;
                if t<=t_pdeacc
                    posSeq(i)=v*t-1/2*amax*t*t;
                    velSeq(i)=v-amax*t;
                else if t<t_pdeacc+t_nacc
                        t=t-t_pdeacc;
                        posSeq(i)=x_0_2_v-1/2*amax*t*t;
                        velSeq(i)=-amax*t;
                    else
                        t=t-t_pdeacc-t_nacc;
                        posSeq(i)=x_0_2_v-1/2*amax*t_nacc*t_nacc-(amax*t_nacc*t-1/2*amax*t*t);
                        velSeq(i)=-amax*t_nacc+amax*t;
                    end
                end
            end
        end
        %如果s负的很多，则需要立即减速，减速到零后反向加速到最大速度，然后匀速，然后减速到0
        if s<=x_0_2_v-2*x_0_2_vm
            t_pacc=0;%正向加速
            t_pconst=0;%正向匀速
            t_pdeacc=t_0_2_v;%正向减速
            t_nacc=t_0_2_vm;%反向加速
            t_nconst=(x_0_2_v-s-2*x_0_2_vm)/vmax;%反向匀速
            t_ndeacc=t_0_2_vm;%反向减速
            
            if(round((t_pacc+t_pconst+t_pdeacc+t_nacc+t_nconst+t_ndeacc)/ts)<=pathSizeMax)
                posSeq=zeros(round((t_pacc+t_pconst+t_pdeacc+t_nacc+t_nconst+t_ndeacc)/ts),1);
                velSeq=zeros(round((t_pacc+t_pconst+t_pdeacc+t_nacc+t_nconst+t_ndeacc)/ts),1);
            else
                posSeq=zeros(pathSizeMax,1);
                velSeq=zeros(pathSizeMax,1);
            end
            for i=1:1:size(posSeq,1)
                t=i*ts;
                if t<=t_pdeacc
                    posSeq(i)=v*t-1/2*amax*t*t;
                    velSeq(i)=v-amax*t;
                    
                else if t<t_pdeacc+t_nacc
                        t=t-t_pdeacc;
                        posSeq(i)=x_0_2_v-1/2*amax*t*t;
                        velSeq(i)=-amax*t;
                        
                    else if t<t_pdeacc+t_nacc+t_nconst
                            t=t-t_pdeacc-t_nacc;
                            posSeq(i)=x_0_2_v-x_0_2_vm-vmax*t;
                            velSeq(i)=-vmax;
                            
                        else
                            t=t-t_pdeacc-t_nacc-t_nconst;
                            posSeq(i)=x_0_2_v-x_0_2_vm-vmax*t_nconst-(vmax*t-1/2*amax*t*t);
                            velSeq(i)=-vmax+amax*t;
                            
                        end
                    end
                end
            end
        end
    else%v>vmax，异常情况
        t_v_2_vm=(v-vmax)/amax;
        x_v_2_vm=(v+vmax)*t_v_2_vm/2;
        t_0_2_vm=vmax/amax;%从0加速至vmax的时间
        x_0_2_vm=1/2*vmax*t_0_2_vm;%从0加速至vmax时走过的路程
        t_0_2_v=v/amax;%从0加速至v的时间
        x_0_2_v=1/2*v*t_0_2_v;
        
    end
end
end

