% NOVA School of Science and Technology
% Department of Electrical and Computer Engineering
% IEEC course, fall 2021
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)
function handle = drone_plot(pos,lbd,handle,dcolor)
% plots a drone given its pose information. 

    n_arms = 4;

    if ~exist('pos','var') || isempty(pos)
        pos = zeros(3,1);
    end
    if ~exist('lbd','var') || isempty(lbd)
        lbd = zeros(3,1);
    end
    if ~exist('handle','var') || isempty(handle)
        handles_arms = zeros(2*n_arms,1);
        handles_body = zeros(1,1);
        is_first = 1;
    else
        handles_arms = handle(1:2*n_arms,:);
        handles_body = handle(2*n_arms+1,:);
        is_first = 0;
    end
    if ~exist('dcolor','var') || isempty(dcolor)
        dcolor = [1,0,0];
    end
    
    internal_rot_axis = Euler2R([pi;0;0]);
    scale = 0.3;
    arm_colors = [ [70,70,70]/255;
                   [200,200,200]/255 ];
        
    rot = Euler2R(lbd)*internal_rot_axis;
    pos = pos';
    
    % draw arms:
    for i = 1:n_arms
        ang = 2*pi/n_arms*(i-1)+pi/4;
        drot = [	cos(ang)	,	 sin(ang)	,	0
                    -sin(ang)	,	 cos(ang)	,	0
                    0			,	 0			,	1	];
        dpos = zeros(1,3);
        handles_arms = draw_square_arm(rot,pos,scale,drot,dpos,arm_colors,handles_arms,i,is_first);
    end
    
    % draw main body:
    handles_body = draw_main_body(rot,pos,scale,dcolor,handles_body,is_first);
    
    handle = [handles_arms;handles_body];

end

function handles = draw_main_body(rot,pos,scale,dcolor,handles,is_first)
    
    r_tri = 0.12*cos(pi/4);
    h_tri = 0.001;
    raw_tri = [ r_tri*cos( 0     ) r_tri*sin( 0     ) -h_tri
                r_tri*cos( 2*pi/3) r_tri*sin( 2*pi/3) -h_tri
                r_tri*cos(-2*pi/3) r_tri*sin(-2*pi/3) -h_tri
                r_tri*cos( 0     ) r_tri*sin( 0     ) -h_tri];
    pts_tri = ((rot*(raw_tri*scale)')' + ...
                kron(ones(size(raw_tri,1),1),pos))';
            
    vert = pts_tri(:,1:3)';
    face = [1 2 3];
    
    if ~is_first
%         set(handles(1,:),'xdata',pts_tri(1,:),'ydata',pts_tri(2,:),'zdata',pts_tri(3,:));
        set(handles(1,:),'Vertices',vert);
    else
%         handles(1,:) = plot3(pts_tri(1,:),pts_tri(2,:),pts_tri(3,:),'LineWidth',0.6,'Color',dcolor);
        handles(1,:) = patch('Faces',face,'Vertices',vert,'FaceColor',dcolor);
    end
    
end

function handles = draw_square_arm(rot,pos,esc,drot,dpos,arm_colors,handles,idx,is_first)

    % arm modeling
    side = 0.005;
    len_ini = 0.0;%7*cos(pi/4);
    len_end = 0.2;            
    raw_pts = [ len_ini     0       0
                len_end     0       0   ];
    pts_arm = ( (rot*( (drot*(raw_pts*esc)')' + ...
            kron(ones(size(raw_pts,1),1),dpos) )')' + ...
            kron(ones(size(raw_pts,1),1),pos) )';
    
    % blade modelling
    t1 = 0:2*pi/7:2*pi;
    r1 = 0.5*len_end;
    h1 = side*1.3;
    l1 = 0.95*len_end;
    raw_props = [ l1+[0,sin(t1)*r1]' , ...
                    [0,cos(t1)*r1]' , ...
                    ones(length(t1)+1,1)*-h1 ];
    pts_props_raw = [];
    for i = 2:length(raw_props)-1
        pts_props_raw = [ pts_props_raw; raw_props(1,:) ; raw_props(i,:) ; raw_props(i+1,:) ];
    end
    pts_props = ( (rot*( (drot*(pts_props_raw*esc)')' + ...
            kron(ones(size(pts_props_raw,1),1),dpos) )')' + ...
            kron(ones(size(pts_props_raw,1),1),pos) )';
    
    if ~is_first
        set(handles(2*idx-1,:),'xdata',pts_arm(1,:),'ydata',pts_arm(2,:),'zdata',pts_arm(3,:));
        set(handles(2*idx,:),'xdata',pts_props(1,:),'ydata',pts_props(2,:),'zdata',pts_props(3,:));
    else
        handles(2*idx-1,:) = plot3(pts_arm(1,:),pts_arm(2,:),pts_arm(3,:),'-','LineWidth',0.4,'Color',arm_colors(1,:));
        handles(2*idx,:) = plot3(pts_props(1,:),pts_props(2,:),pts_props(3,:),'-','LineWidth',0.1,'Color',arm_colors(2,:));
    end
      
end