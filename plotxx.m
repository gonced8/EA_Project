function [ax,hl1,hl2] = plotxx(fig,x1,y1,x2,y2,y3,xlabels,ylabels)
%PLOTXX - Create graphs with x axes on both top and bottom 
%
%Similar to PLOTYY, but ...
%the independent variable is on the y-axis, 
%and both dependent variables are on the x-axis.
%
%Syntax: [ax,hl1,hl2] = plotxx(x1,y1,x2,y2,xlabels,ylabels);
%
%Inputs:  X1,Y1 are the data for the first line (black)
%         X2,Y2 are the data for the second line (red)
%         XLABELS is a cell array containing the two x-labels
%         YLABELS is a cell array containing the two y-labels
%
%The optional output handle graphics objects AX,HL1,HL2
%allow the user to easily change the properties of the plot.
%
%Example: Plot temperature T and salinity S 
%         as a function of depth D in the ocean
%
%D = linspace(-100,0,50);
%S = linspace(34,32,50);
%T = 10*exp(D/40);
%xlabels{1} = 'Temperature (C)';
%xlabels{2} = 'Salinity';
%ylabels{1} = 'Depth(m)';
%ylabels{2} = 'Depth(m)';
%[ax,hlT,hlS] = plotxx(T,D,S,D,xlabels,ylabels);
%The code is inspired from page 10-26 (Multiaxis axes)
%of the manual USING MATLAB GRAPHICS, version 5.
%
%Tested with Matlab 5.3.1 and above on PCWIN
%Author: Denis Gilbert, Ph.D., physical oceanography
%Maurice Lamontagne Institute, Dept. of Fisheries and Oceans Canada
%email: gilbertd@dfo-mpo.gc.ca  Web: http://www.qc.dfo-mpo.gc.ca/iml/
%November 1997; Last revision: 01-Nov-2001
if nargin < 6
   error('Not enough input arguments')
elseif nargin==6
   %Use empty strings for the xlabels
   xlabels{1}=' '; xlabels{2}=' '; ylabels{1}=' '; ylabels{2}=' ';
elseif nargin==7
   %Use empty strings for the ylabel
   ylabels{1}=' '; ylabels{2}=' ';
elseif nargin > 8
   error('Too many input arguments')
end
if length(ylabels) == 1
   ylabels{2} = ' ';
end
if ~iscellstr(xlabels) 
   error('Input xlabels must be a cell array')
elseif ~iscellstr(ylabels) 
   error('Input ylabels must be a cell array')
end

figure(fig);
size_y = [min([min(y1), min(y2), y3]), max([max(y1), max(y2), y3])];
size_y = size_y + diff(size_y)*0.05*[-1, 1];

hl1=line(x1,y1,'Color','k');
ax(1)=gca;
%set(ax(1),'Position',[0.12 0.12 0.75 0.70])
set(ax(1),'XColor','k','YColor','k');
ax(2)=axes('Position',get(ax(1),'Position'),...
   'XAxisLocation','top',...
   'YAxisLocation','right',...
   'Color','none',...
   'XColor','r','YColor','k');
set(ax,'box','off')
hl2=line(x2,y2,'Color','r','Parent',ax(2));
%label the two x-axes
set(get(ax(1),'xlabel'),'string',xlabels{1},'interpreter','latex')
set(get(ax(2),'xlabel'),'string',xlabels{2},'interpreter','latex')
set(get(ax(1),'ylabel'),'string',ylabels{1},'interpreter','latex')
set(get(ax(2),'ylabel'),'string',ylabels{2},'interpreter','latex')

ylim(ax(1), size_y);
ylim(ax(2), size_y);
axes(ax(1)); hold on;
axes(ax(2)); hold on;
end
