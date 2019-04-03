
x0=.3; 
y0=.05; 
sigma=1; 
x=.1:.07:.3;
y=-.2:.07:.1;

X=NaN*zeros(i*j,2);  % init
count=0;
for i=1:length(x), 
  for j=1:length(y), 
    count=count+1; 
    X(count,:)=[x(i) y(j)];
  end; 
end;  
X;

plot3(X(i,1),X(i,2),x0+exp(-X(:,1).^2/sigma^2),'.'),
