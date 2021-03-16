function []= graph2d(phi,char0,sumz,char1,char2)
figure
j=1;
if char0=='phi1'
for i=1:13%length(phi)
    subplot(4,4,i)
    plot(phi,sumz(:,j))
    xlim([0 360]);
    ylim([-50 50]);
    xlabel(char0);
    ylabel(char1);
    a=num2str((j-1)*10);
    str=append(char2,'=',a);
    title(str)
    j=j+3;
end
else
    for i=1:13%length(phi)
    subplot(4,4,i)
    plot(phi,sumz(j,:))
    xlim([0 360]);
    ylim([-50 50]);
    xlabel(char0);
    ylabel(char1);
    a=num2str((j-1)*10);
    str=append(char2,'=',a);
    title(str)
    j=j+3;
    end
    
end
end

