clf
% Subscribe Bottom raw image
ForwardVideo = rossubscriber('/ardrone/bottom/image_raw'); 
msgtype = rostype.geometry_msgs_Twist;
pub = rospublisher('/cmd_vel', msgtype);

% Publish on /ardrone/land topic to land
chatpub2= rospublisher('/ardrone/land','std_msgs/Empty');
msg2 = rosmessage(chatpub2);

% Publish on /ardrone/takeoff topic to takeoff
chatpub= rospublisher('/ardrone/takeoff','std_msgs/Empty');
msg3 = rosmessage(chatpub);

send(chatpub,msg3); % takeoff at start 

Video = receive(ForwardVideo,1.5); %create video object to receive data with 1.5s timeout
img =readImage(Video); %image objectn to read image data

%convert to gray scale image
gray = rgb2gray(img); 

%convert to binary image
threshold = 10; %to detect black
BW = im2bw(gray,threshold/255);
clf
imshow(BW,'Border','tight')

%find slope & intercet of the line
[y, x] = find(BW); 
coeff = polyfit(x,y,1);
slop = coeff(1);
intercept = coeff(2);

message = sprintf('slope is %f',slop) %print values

area1 = bwarea(BW);

pause(3);

while 1 %continuous loop
    Video = receive(ForwardVideo,1);%create video object to receive data with 1.5s timeout
    img =readImage(Video);%image objectn to read image data
    
    %image objectn to read image data
    gray = rgb2gray(img);
    
    %convert to binary image
    threshold = 10;
    BW = im2bw(gray,threshold/255);
    clf
    imshow(BW,'Border','tight')
    
    %find slope & intercet of the line
    [y, x] = find(BW); 
    coeff = polyfit(x,y,1);
    slo = coeff(1);
    intercept = coeff(2);
    message = sprintf('slope is %f',slo);
    message = sprintf('slope difference %f',slop-slo)

    
    area2 = bwarea(BW); 
    message = sprintf('area difference %f',area2-area1)
    
    %slo has current slope and slop has starting slope
    %calculate difference between both will reduce deviation and help to
    %follow the line
    if round(slop-slo,4)~=0.0000 && round(slop-slo,4)~=-0.0000 
        if slop-slo>-0.001 && slop-slo<0.001 %move forward
            msg = rosmessage(msgtype);
            msg.Linear.X = 1; %linear movement 1 m/s
            send(pub,msg);
            disp('forward')
        
        elseif slop-slo<-0.001 %turn left
            msg = rosmessage(msgtype);
            msg.Linear.X = 0;
            msg.Linear.Y = 0;
            %msg.Angular.Z = 0.35; %turn 0.35 radians per sec
            msg.Angular.Z = 0.15;
            send(pub,msg);
            disp('left')
            delay = 0-slo;
            pause(delay)
        elseif slop-slo>0.001 %turn right
            msg = rosmessage(msgtype);
            msg.Linear.X = 0;
            msg.Linear.Y = 0;
            %msg.Angular.Z = -0.35;%turn 0.35 radians per sec
            msg.Angular.Z = -0.15;
            send(pub,msg);
            disp('right')
            pause(slo)
        end
    elseif (round(area2-area1)>220000) && slop-slo~=-0.000042%stop %check if there is no line then land
        send(chatpub2,msg2);
        disp('land')
        message = sprintf('area',bwarea(BW))
        hold
    end
end