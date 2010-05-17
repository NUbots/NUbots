dt = 0.01;              # set this to the motion frame period in seconds
R = 1*10^(-6);
N = 120;                # set this the number of preview frames (approx: 1.2 seconds worth)
numPreviewFrames = N;
Qx = 0.25;  Qe = 0.3;
Ql = [1,0,0;0,1,0;0,0,1];

g = 9800;               # gravity in cm/s/s
z_h = 230;              # CoM height in mm

A0 = [1, dt, 0; g/z_h*dt, 1, -g/z_h*dt; 0, 0, 1];
b0 = [0; 0; dt];
c0 = [0, 0, 1];

Bt(1,1)=c0*b0;
Bt(2:4,1)=b0(1:3);
It(1,1)=1;
It(2:4,1)=0;
Ft(1,1:3)=c0*A0;
Ft(2:4,1:3)=A0(1:3,1:3);
Qt(1:4, 1:4)=0;
Qt(1,1)=Qe;
Qt(2:4,2:4)=c0' *Qx*c0;
At(1:4,1)=It;
At(1:4,2:4)=Ft;


Pt=dare(At, Bt, Qt, R);

Gx = (1/(R+Bt'*Pt*Bt)) * Bt'*Pt*Ft;
Gi = (1/(R+Bt'*Pt*Bt)) * Bt'*Pt*It;

Ac = At - Bt*(1/(R + Bt'*Pt*Bt)) * Bt'*Pt*At;
X = -Ac'*Pt*It;
Gd(1) = -Gi;
for i=2:N,
  Gd(i) = (1/(R + Bt'*Pt*Bt))*Bt'*X;
  X = Ac'*X;
end

A = A0-b0*Gx;

L = dlqr(A', c0', Ql , R)';

endTime = 10;
for time=[0:dt:endTime],
  preview_frames = [];

  for j=[1:numPreviewFrames],
    preview_frames(j) = 0.1*real( Gd(j)*(time + j*dt) );
  end;
end;
              
fid = fopen('~/Desktop/preview_frames.txt', 'w')
fprintf(fid, "const float Observer::weights[NUM_AVAIL_PREVIEW_FRAMES] = \n{")
fprintf(fid, "%f, %f, %f, %f, %f, %f, %f, %f,\n", preview_frames)
fprintf(fid, "};\n\n")

fprintf(fid, "const float Observer::A_values[9] = \n{")
fprintf(fid, "%f, %f, %f,\n", A')
fprintf(fid, "};\n\n")
              
fprintf(fid, "const float Observer::b_values[3] = {")
fprintf(fid, "%f, %f, %f", b0)
fprintf(fid, "};\n\n")
              
fprintf(fid, "const float Observer::L_values[3] = {")
fprintf(fid, "%f, %f, %f", L)
fprintf(fid, "};\n\n")
              
fprintf(fid, "const float Observer::c_values[3] = {")
fprintf(fid, "%f, %f, %f", c0)
fprintf(fid, "};\n\n")
              
fprintf(fid, "const float Observer::Gi = %f;\n", Gi)          

fclose(fid)
