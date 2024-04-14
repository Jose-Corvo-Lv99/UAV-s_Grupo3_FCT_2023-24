% NOVA School of Science and Technology
% Department of Electrical and Computer Engineering
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)
function rot = Euler2R(ang)
%	Converts an euler angles vector [phi;theta;psi] into a rotation matrix
%	of the Z-Y-X representation: R = rot_z(psi)*rot_y(theta)*rot_x(phi).

rotx = [	1	,	0			,	 0
			0	,	cos(ang(1))	,	-sin(ang(1))
			0	,	sin(ang(1))	,	 cos(ang(1))	];
roty = [	 cos(ang(2))	,	0	,	sin(ang(2))
			0				,	1	,	0
			-sin(ang(2))	,	0	,	cos(ang(2))	];
rotz = [	cos(ang(3))	,	-sin(ang(3))	,	0
			sin(ang(3))	,	 cos(ang(3))	,	0
			0			,	 0				,	1	];
rot = rotz*roty*rotx;

end