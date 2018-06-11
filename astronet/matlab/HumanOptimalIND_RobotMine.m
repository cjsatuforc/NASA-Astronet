
while K>0
%Initialize prior belief of next human goal
for k=1:K
dist_h_g(k)=1/norm((Human(t,:)-goal(k,:)));
end
P_g_s=dist_h_g./sum(dist_h_g(:));
P_g_s=transpose(P_g_s);

%Compute every possible path through the network and it's path length
    paths=perms(1:K);
    paths(:,K+1)=0;
    paths(:,K+2)=sqrt((goal(paths(:,1),1)-Human(t,1)).^2+(goal(paths(:,1),2)-Human(t,2)).^2);
    for j=1:K-1
    paths(:,K+1)=paths(:,K+1)+sqrt((goal(paths(:,j),1)-goal(paths(:,j+1),1)).^2+(goal(paths(:,j),2)-goal(paths(:,j+1),2)).^2);
    %paths K+2j is human_collab cost and K+2j+1 is robot collab cost
    %j=1 corresponds to human with first task and robot with all others, 
    if j<K-1
    paths(:,K+2*(j+1))=paths(:,K+2*j)+sqrt((goal(paths(:,j+1),1)-goal(paths(:,j),1)).^2+(goal(paths(:,j+1),2)-goal(paths(:,j),2)).^2);
    end
    end
    for j=1:K-1
    paths(:,K+2*j+1)=paths(:,K+2*j-1)-sqrt((goal(paths(:,j+1),1)-goal(paths(:,j),1)).^2+(goal(paths(:,j+1),2)-goal(paths(:,j),2)).^2);
    end
    for j=1:K-1
    paths(:,K+2*j+1)=paths(:,K+2*j+1)+sqrt((goal(paths(:,j+1),1)-Robot(t,1)).^2+(goal(paths(:,j+1),2)-Robot(t,2)).^2);
    end

  
  for j=1:K-1
  possible_collab_lengths(:,j)=max(paths(:,K+2*j),paths(:,K+2*j+1));
  end
  %change this to have K possible Robot goals instead and select them
  %online later
  if K>1
  Robot_Goals=ones(K,K-1);
  for k=1:K
  possible_collab_temp=possible_collab_lengths((k-1)*factorial(K-1)+1:k*factorial(K-1),:);
  [min_val,idx]=min(possible_collab_temp(:));
  
  idx=idx+ceil(idx/factorial(K-1))*(k-1)*factorial(K-1)+(ceil(idx/factorial(K-1))-1)*(K-k)*factorial(K-1);
  
  [Sequence,Collab_point]=ind2sub(size(possible_collab_lengths),idx);
  Robot_Goals(k,1:length(paths(Sequence,Collab_point+1:K)))=paths(Sequence,Collab_point+1:K)
  end

  
  else
  Robot_Goals=1;    
  end

while(1)    
    Human_feas_zone=zeros(y_max,x_max);
    for x=1:x_max
        for y=1:y_max
       if norm([x-Human(t,1),y-Human(t,2)])<=H_max
          Human_feas_zone(y,x)=1; 
       end
        end
    end
    %Cycle through each goal
    for k=1:K
    P_a_sg(:,:,k)=exp(Beta.*(gamma.^(sqrt((x_mesh-goal(k,1)).^2+(y_mesh-goal(k,2)).^2))*R-c*((gamma-gamma.^(sqrt((x_mesh-goal(k,1)).^2+(y_mesh-goal(k,2)).^2)))/(1-gamma))));
    P_a_sg(:,:,k)=squeeze(P_a_sg(:,:,k)).*Human_feas_zone;
    P_a_sg(:,:,k)=P_a_sg(:,:,k)/sum(sum(P_a_sg(:,:,k)));
    end
    %Human moves 1 unit towards goal
    Human(t+1,1)=(x_head-x_orig)*10;
    Human(t+1,2)=(y_head-y_orig)*10;
    %Now Update Distribution
    P_g_s=squeeze(P_a_sg(max(1,min(int64(Human(t+1,2)),y_max)),max(1,min(int64(Human(t+1,1)),x_max)),:)).*P_g_s;
    P_g_s=P_g_s./sum(P_g_s)
    
    Robot_Goal_P=zeros(K,1);
    for k=1:K
     %Robot_Goal_P(opt_path(k,K))=Robot_Goal_P(opt_path(k,K))+P_g_s(k);
     Robot_Goal_P(Robot_Goals(k,1))=Robot_Goal_P(Robot_Goals(k,1))+P_g_s(K+1-k);
    end
    Robot_feas_zone=inf*ones(y_max,x_max);
    
    for x=1:x_max
        for y=1:y_max
       if norm([x-Robot(t,1),y-Robot(t,2)])<=R_max
          Robot_feas_zone(y,x)=1; 
       end
        end
    end
    Robot_opt_map=zeros(y_max,x_max);
    normalizer=0;
    for k=1:K
    Robot_opt_map=Robot_opt_map+(max(0,(-log(sqrt(((Robot(t,1)-goal(k,1)).^2+(Robot(t,2)-goal(k,2)).^2))./att_rad)))+Robot_Goal_P(k)).*sqrt((x_mesh-goal(k,1)).^2+(y_mesh-goal(k,2)).^2);
    normalizer=normalizer+((x_mesh-goal(k,1)).^2+(y_mesh-goal(k,2)).^2);
    end
    %Robot_opt_map=Robot_opt_map./normalizer;
    Robot_opt_map=Robot_opt_map.*Robot_feas_zone;
    [garbage,rob_dex]=min(Robot_opt_map(:));
    [y_rob_dest,x_rob_dest]=ind2sub(size(Robot_opt_map),rob_dex);
    %Propagate Robot State
    Robot(t+1,1)=x_rob_dest;
    Robot(t+1,2)=y_rob_dest;

t=t+1;

%Check capture of target and break infinite loop
if min(sqrt((Human(t,1)-goal(:,1)).^2+(Human(t,2)-goal(:,2)).^2))<cap_tol || min(sqrt((Robot(t,1)-goal(:,1)).^2+(Robot(t,2)-goal(:,2)).^2))<cap_tol

    if min(sqrt((Human(t,1)-goal(:,1)).^2+(Human(t,2)-goal(:,2)).^2))<cap_tol
         
        [garbage,dex]=min(sqrt((Human(t,1)-goal(:,1)).^2+(Human(t,2)-goal(:,2)).^2));
        %[x position, y position, agent 1, time]
        Capture_List=[Capture_List;goal(dex,:),1,t];
        goal(dex,:)=[];
        K=K-1;
       
        
    end
    
    if min(sqrt((Robot(t,1)-goal(:,1)).^2+(Robot(t,2)-goal(:,2)).^2))<cap_tol
        
        [garbage,dex]=min(sqrt((Robot(t,1)-goal(:,1)).^2+(Robot(t,2)-goal(:,2)).^2));
        %[x position, y position, agent 2, time]
        Capture_List=[Capture_List;goal(dex,:),2,t];
        goal(dex,:)=[];
        K=K-1;
    end
clear dist_h_g P_g_s paths opt_dex opt_path P_a_sg possible_collab_lengths possible_collab_temp Robot_Goals
break
end
    
end

Final_Time=Capture_List(end,end)
%End of outer loop
end
