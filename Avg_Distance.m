function d=Avg_Distance(a,mx,my)
d=0;
[x,y]=size(a);
for i=1:x
    d=d+sqrt((a(i,1)-mx)*(a(i,1)-mx) + (a(i,2)-my)*(a(i,2)-my));
end
d=d/x;
end
    