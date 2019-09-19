function [] = figure_saver_TOS(path)
%FFIGURE_SAVER_TOS Summary of this function goes here
%   Detailed explanation goes here

    FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
    for iFig = 1:length(FigList)
      FigHandle = FigList(iFig);
      FigName   = num2str(get(FigHandle, 'name'));
      set(0, 'CurrentFigure', FigHandle);
      savefig(fullfile(path, [FigName '.fig']));
    end

end

