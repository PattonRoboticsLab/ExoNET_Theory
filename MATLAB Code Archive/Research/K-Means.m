% kmeans_randexample2.m
%%Randomly generate the sample data.

points=500; %define how many datapoints you want. 

rng default; % For reproducibility this starts the random generator the same way (same seed) each time the code is run.
X = [B;     % This is a randomly generated dataset, randn creates a random column of data points long with two columns for x and y values to plot later.
    C;
    D];    % This points x 2 size data matrix has one set of points with 1 added and one set with 1 subtracted, the constant multiplying randn can be altered to change distance between the two groups of points.
%can make these overlap with 0.8, 0.6; imperfect sorting

figure;                         % plots the generated data for you
scatter3(B,C,D,'.');
title 'Randomly Generated Data';
%% Partition the data into two clusters, and choose the best arrangement out of five intializations. Display the final output.

numclusts=2;

opts = statset('Display','final');  % This is just for showing you that iterations of the k-means converge to the same minimal distance
[idx,C] = kmeans(X,numclusts,'Distance','cityblock',...
   'Replicates',5,'Options',opts); %puts centroid locations in C

%[idx,C] = kmeans(X,numclusts);  % This is the standard kmeans used on X with 2 clusters (line 16) and euclidian distance by default.

%scatter3 gives 3D scatterplot
%in idX everything has value of 1 or 2
%% Plot the clusters and the cluster centroids.

figure;
scatter3(X(idx==1,1),X(idx==1,2),X(idx==1,3),'r.','MarkerEdgeAlpha',0.3);
hold on
scatter3(X(idx==2,1),X(idx==2,2),X(idx==2,3),'b.','MarkerEdgeAlpha',0.3);

scatter3(C(:,1),C(:,2),C(:,3),'kx',’LineWidth’,5)
legend('Cluster 1','Cluster 2','Centroids',...
       'Location','NW')
title 'Cluster Assignments and Centroids'
hold off

Replicate 1, 1 iterations, total sum of distances = 771.19.
Replicate 2, 1 iterations, total sum of distances = 771.19.
Replicate 3, 1 iterations, total sum of distances = 771.19.
Replicate 4, 1 iterations, total sum of distances = 771.19.
Replicate 5, 1 iterations, total sum of distances = 771.19.
Best total sum of distances = 771.19