function [posSeq,velSeq] = TrapezoidalPathGen(s,v,ts,vmax,amax,pathSizeMax)
%�����η�������ٶȡ������ٶȹ滮·��
%   s:���λ��
%   v:��ǰ�ٶ�
%   ts:��������
%   vmax:����ٶ�
%   amax:�����ٶ�

if v<0
    [posSeq_,velSeq_]=TrapezoidalPathGen(-s,-v,ts,vmax,amax,pathSizeMax);
    posSeq=-posSeq_;
    velSeq=-velSeq_;
else
    %%
    %ֻ�迼��v>0�����
    if v<=vmax%���vС��vmax���������
        t_0_2_vm=vmax/amax;%��0������vmax��ʱ��
        x_0_2_vm=1/2*vmax*t_0_2_vm;%��0������vmaxʱ�߹���·��
        t_v_2_vm=(vmax-v)/amax;%��v������vmax��ʱ��
        x_v_2_vm=1/2*(vmax+v)*t_v_2_vm;%��v������vmaxʱ�߹���·��
        t_0_2_v=v/amax;%��0������v��ʱ��
        x_0_2_v=1/2*v*t_0_2_v;
        
        
        %���s�㹻������·���Ǽ��ٵ����Ȼ�����٣�Ȼ����ٵ�0
        if s>x_v_2_vm+x_0_2_vm
            t_pacc=t_v_2_vm;%�������
            t_pconst=(s-x_v_2_vm-x_0_2_vm)/vmax;%��������
            t_pdeacc=t_0_2_vm;%�������
            t_nacc=0;%�������
            t_nconst=0;%��������
            t_ndeacc=0;%�������
            
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
        %���s�Զ̣����ȼ���Ȼ����ٵ�0
        if s<=x_v_2_vm+x_0_2_vm && s>x_0_2_v
            t_pacc=sqrt((s+x_0_2_v)/amax)-t_0_2_v;%�������
            t_pconst=0;%��������
            t_pdeacc=sqrt((s+x_0_2_v)/amax);%�������
            t_nacc=0;%�������
            t_nconst=0;%��������
            t_ndeacc=0;%�������
            
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
        %���s�ܶ�����Ϊ��������Ҫ�������٣����ٵ��������٣�Ȼ����ٵ�0
        if s>x_0_2_v-2*x_0_2_vm && s<=x_0_2_v
            t_pacc=0;%�������
            t_pconst=0;%��������
            t_pdeacc=t_0_2_v;%�������
            t_nacc=sqrt((x_0_2_v-s)/amax);%�������
            t_nconst=0;%��������
            t_ndeacc=sqrt((x_0_2_v-s)/amax);%�������
            
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
        %���s���ĺܶ࣬����Ҫ�������٣����ٵ��������ٵ�����ٶȣ�Ȼ�����٣�Ȼ����ٵ�0
        if s<=x_0_2_v-2*x_0_2_vm
            t_pacc=0;%�������
            t_pconst=0;%��������
            t_pdeacc=t_0_2_v;%�������
            t_nacc=t_0_2_vm;%�������
            t_nconst=(x_0_2_v-s-2*x_0_2_vm)/vmax;%��������
            t_ndeacc=t_0_2_vm;%�������
            
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
    else%v>vmax���쳣���
        t_v_2_vm=(v-vmax)/amax;
        x_v_2_vm=(v+vmax)*t_v_2_vm/2;
        t_0_2_vm=vmax/amax;%��0������vmax��ʱ��
        x_0_2_vm=1/2*vmax*t_0_2_vm;%��0������vmaxʱ�߹���·��
        t_0_2_v=v/amax;%��0������v��ʱ��
        x_0_2_v=1/2*v*t_0_2_v;
        
    end
end
end

