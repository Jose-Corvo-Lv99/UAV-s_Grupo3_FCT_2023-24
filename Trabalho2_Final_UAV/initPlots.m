%% inicializations for visualization (you can ignore this)
sstdarkblue     = [0,73,219]/255;
sstblue         = [0,128,255]/255;
sstlightblue    = [48,208,216]/255;
sstgreen        = [43,191,92]/255;
sstlightgreen   = [140,255,200]/255;
sstgray         = [70,70,70]/255;
sstlightgray    = [200,200,200]/255;
newColorOrder = [sstblue;sstlightblue;sstgreen;sstlightgreen;sstdarkblue;sstgray;sstlightgray];
try % matlab only
  set(groot,'defaultTextInterpreter','latex');
  set(groot,'defaultLegendInterpreter','latex');
  set(groot,'defaultLineLineWidth',2)
  set(groot,'defaultAxesColorOrder',newColorOrder)
end

try %octave only
  pkg load control
end




