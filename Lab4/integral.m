function result = integral(t_init, t_final, velocity)

N = 1000;
h = (t_final-t_init)/N;
result = 0;
for i = 1:N
    result = result + (velocity);
end

result = h * result;
    