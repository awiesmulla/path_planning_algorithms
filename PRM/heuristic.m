function h=heuristic(X,goal)
h = sqrt(sum((X-goal).^2));