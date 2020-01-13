function J = objective(u)

J = zeros(length(u),1).';
for i = 1:length(u)
    J(i) = norm(u(:,i))^2;
end
end