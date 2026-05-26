import sympy as sp

m1,m2,c2,s2, r, z = sp.symbols('m1 m2 c2 s2 r z')

eq1 = sp.Eq(m1*c2 - m2*s2 , r)
eq2 = sp.Eq(m1*s2 + m2*c2 , z)

print(sp.solve([eq1, eq2], (c2,s2)))

tub = (i*i for i in range(5))
print(tub)
print(sum(tub))
print(tub)
print(sum(tub))

[100,50,100;50,100,100;-100,50,100;50,-100,100]