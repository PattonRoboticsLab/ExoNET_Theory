% ********************************************************************************
% Place a figure on the screen based on the fractional dimensions of pixel space
%
% put_figure(fignum, left, up, width, height)
%       fignum = index number of the figure
%       left = left edge of figure from left edge of screen as
%              a fraction of screen width
%       up = bottom edge of figure from bottom edge of screen as
%            a fraction of screen height
%       width = figure width as a fraction of screen width
%       height = figure height as a fraction of screen height
% ********************************************************************************

function put_figure(fignum,left,up,width,height)

global DEBUGIT

if DEBUGIT
    fprintf(' ~~ put_figure.M ~~')
end

screen_size = get(0,'ScreenSize'); % dimensions of the screen in pixels
screen_width = screen_size(3);     % screen width in pixels
screen_height = screen_size(4);    % screen height in pixels

figure(fignum)
set(fignum,'Position',[left*screen_width up*screen_height width*screen_width height*screen_height]); % [Xleft Ybottom width height]

if DEBUGIT
    fprintf(' ~~ END put_figure ~~');

end