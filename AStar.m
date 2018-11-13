
%% A star Planning
%    A-start is used to find the shortest distance between the start and the goal. 
%    The heuristic used to evaluate the shortest distances in A* is: 
%     *f(n) = g(n) + h(n)*
%
%   h(n) is the euclidean distance between the point of interest and the goal 
%   g(n) is defined as the cost of moving from the start point to the current position 
%   f(n) is the total distance, i.e., the sum of G and H 
%%
% Demo :<https://www.youtube.com/watch?v=CFD1cfQcOWU>

%% A* Pseudocode 
 
%% Step 1: Initialization of the Values
%Initialize the start point (xStart, yStart), goal point (xGoal, yGoal) and
%obstacles (blocks).
xStart=[1 9]; %First Start point
yStart=[9 1]; %Second Start point

for start = 1:2
%Initialize the Grid
xLength=12; yLength=12;
grid1=(ones(xLength,yLength));
axis([1 12 1 12])
grid on;
ax = gca;
ax.XColor = 'b'; 
ax.YColor = 'b';
ax.GridColor = 'b';
ax.GridAlpha = 0.9;
ax.Colormap = [1 0 1; 0 0 1; 1 1 0];
hold on;
title('A* Planning');
block = [1 3; 2 2; 3 4; 5 9;4 3; 8 3; 4 5];

grid1(xStart(start),yStart(start))=1;
plot(xStart(start)+.5,yStart(start)+.5,'o','MarkerSize',20,...
        'MarkerEdgeColor','red',...
        'MarkerFaceColor','red');    

%Setting up the goal (xGoal, yGoal)
xGoal=6;
yGoal=3;
grid1(xGoal,yGoal)=0;
plot(xGoal+.5,yGoal+.5,'o','MarkerSize',20,...
    'MarkerEdgeColor','blue',...
        'MarkerFaceColor','blue');

%Setting up the Obstacles (block)
for Obs = 1: 1: length(block)
grid1(block(Obs,1),block(Obs,2))=-1;
end
plot(block(:,1)+0.5,block(:,2)+0.5,'s','MarkerSize',15,...
       'MarkerEdgeColor','black',...
        'MarkerFaceColor','black');
x_val = 1;
y_val = 1;   
a=1;j=0;

%% Step 2 : Create two lists
openList=[];
closedList=[];

%Setting up the closedList    
for i=1:xLength
    for j=1:yLength
        if(grid1(i,j) == -1)
            closedList(a,1)=i; 
            closedList(a,2)=j; 
            a=a+1;
        end
    end
end


countClose=size(closedList,1);
xPosition=xStart(start);
yPosition=yStart(start);
countOpen=1;
G=0;

%% Step 4 Set H value based on the distance 

hDis=sqrt((xPosition-xGoal)^2+(yPosition-yGoal)^2);
%formula to find the distance between two points

%% Step 5 Moving the start point to closedList

openList(countOpen,:)=[1,xPosition,yPosition,xPosition,yPosition,G,hDis,(hDis+G)];
openList(countOpen,1)=0;
countClose=countClose+1;
closedList(countClose,1)=xPosition;
closedList(countClose,2)=yPosition;
Flag=1;

%% Step 6a. Find the next positions to be opened (Checking if they are already reached or if they are blocked)

%Creating the loop to find the F value to reach the goal
while((xPosition ~= xGoal || yPosition ~= yGoal) && Flag == 1)
    NewA=[];
    count_e=1;
    c2=size(closedList,1);
    for act=1:4
            switch(act)
                case 1
                New_ValA = xPosition+1;
                New_ValB = yPosition;
                    
                case 2
                New_ValA = xPosition-1;
                New_ValB = yPosition;
                    
                case 3
                New_ValA = xPosition;
                New_ValB = yPosition+1;
                
                case 4
                New_ValA = xPosition;
                New_ValB = yPosition-1;
            
            end
                    
                if( (New_ValA > 0 && New_ValA <= xLength) && (New_ValB > 0 && New_ValB <= yLength))
                    flag1=1;                    
                    for c1=1:c2
                        if(New_ValA == closedList(c1,1) && New_ValB == closedList(c1,2))
                            flag1=0;
                            
                        end
                    end
                    if (flag1 == 1)
                        NewA(count_e,1) = New_ValA; NewA(count_e,2) = New_ValB;
                        NewA(count_e,3) = G+sqrt((xPosition-New_ValA)^2+(yPosition-New_ValB)^2);
                        NewA(count_e,4) = sqrt((xGoal-New_ValA)^2+(yGoal-New_ValB)^2);
                        NewA(count_e,5) = NewA(count_e,3)+NewA(count_e,4);
                        count_e=count_e+1;
                       
                    end
                end
    end
    
  %% Step 6b. Based on the previous step, move the suitable points to the openList
 count_e=size(NewA,1);
 for i=1:count_e
    flag=0;
    for j=1:countOpen
        if(NewA(i,1) == openList(j,2) && NewA(i,2) == openList(j,3) )
            openList(j,8)=min(openList(j,8),NewA(i,5));
            if openList(j,8)== NewA(i,5)
                openList(j,4)=xPosition;
                openList(j,5)=yPosition;
                openList(j,6)=NewA(i,3);
                openList(j,7)=NewA(i,4);
            end
            flag=1;
        end
    end
    if flag == 0
        countOpen = countOpen+1;
        openList(countOpen,:)=[1,NewA(i,1),NewA(i,2),xPosition,yPosition,NewA(i,3),NewA(i,4),NewA(i,5)];
    end
 end

 %% Step 7a Find the lowest F value for the points in the openList
 
 isOK=[];
 a=1;
 n_f=0;
 I_goal=0;
 for j=1:countOpen
     if (openList(j,1)==1)
         isOK(a,:)=[openList(j,:) j]; 
         if (openList(j,2)==xGoal && openList(j,3)==yGoal)
             n_f=1;
             I_goal=j;
         end
         a=a+1;
     end
 if n_f == 1 
     V_Val=I_goal;
 end    
 end
 if size(isOK ~= 0)
  [min_fn,temp_min]=min(isOK(:,8));
  V_Val=isOK(temp_min,9);
 else
     V_Val=-1;
 end
 
  ValN = V_Val;
  if (ValN ~= -1)    
      
%       
%% Step 7b Choose the above as the next position
   
   xPosition=openList(ValN,2);
   yPosition=openList(ValN,3);
   G=openList(ValN,6);
  
   %% Move this point to the closedList and if this is equivalent to the goal, then terminate
   
  countClose=countClose+1;
  closedList(countClose,1)=xPosition;
  closedList(countClose,2)=yPosition;
  openList(ValN,1)=0;
  else
        Flag=0;
  end
end

i=size(closedList,1);
f_Val=[];
xval=closedList(i,1);
yval=closedList(i,2);
i=1;
f_Val(i,1)=xval;
f_Val(i,2)=yval;
i=i+1;
if ((xval == xGoal) && (yval == yGoal)) 
   q1=0;
   q2=1;
    while(openList(q2,2) ~= xval || openList(q2,3) ~= yval )
        q2=q2+1;
    end
    n_index=q2;
   New_x=openList(n_index,4);
   New_y=openList(n_index,5);
   while( New_x ~= xStart(start) || New_y ~= yStart(start))
           f_Val(i,1) = New_x;
           f_Val(i,2) = New_y;
           
                N_i=1;
                while(openList(N_i,2) ~= New_x || openList(N_i,3) ~= New_y )
                    N_i=N_i+1;
                end
                q1=N_i;
          
           New_x=openList(q1,4);
           New_y=openList(q1,5);
           i=i+1;
   end
  %% Display the F, G and H value 
 j=size(f_Val,1);
 p=plot(f_Val(j,1)+.5,f_Val(j,2)+.5,'o','MarkerSize',20,...
        'MarkerEdgeColor','red'); %optimal path found    
    for oplen=1:length(openList)
                 FGHValue=['G:', num2str(openList(oplen,6)),'  H:', num2str(openList(oplen,7)),'  F:', num2str(openList(oplen,8))]
               
    end  
 j=j-1;
 for i=j:-1:1
  pause(.60);
  set(p,'XData',f_Val(i,1)+.5,'YData',f_Val(i,2)+.5);
 drawnow ;
 end
 
 %To plot the optimal path
    plot(f_Val(:,1)+.5,f_Val(:,2)+.5,'*','MarkerSize',10,...
       'MarkerEdgeColor','red',...
        'MarkerFaceColor','red');
       
else
disp('ERROR');
end
%% 
pause(5);
close;
end





