function print2pdf(outfilename,doprint)
% https://stackoverflow.com/questions/3801730/get-rid-of-the-white-space-around-matlab-figures-pdf-output
    if exist('doprint','var') && doprint
        h = gcf;
        set(h, 'PaperUnits','centimeters');
        set(h, 'Units','centimeters');
        pos=get(h,'Position');
        set(h, 'PaperSize', [pos(3) pos(4)]);
        set(h, 'PaperPositionMode', 'manual');
        set(h, 'PaperPosition',[0 0 pos(3) pos(4)]);

        print('-dpdf',outfilename);
    end

end

