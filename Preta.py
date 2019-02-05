# -*- coding: utf-8 -*-
"""
Created on Tue Jan  1 09:45:22 2019

@author: jucem
"""
import matplotlib.pyplot as plt


def Pr(Partida, Destino, Atual):
    xa , ya = Partida
    xb , yb = Destino
    x , y = Atual
    d = abs( ( ya-yb)*x + (xa - xb)*y + xa*yb - xb - ya )/ ( ((ya-yb)**2) + ((xa - xb)**2) )**(1/2)
    return d

Partida = (9,2)
Destino = (3,12)
Atual = (9,2) # Vai ficar sempre atualizando 

xa , ya = Partida
xb , yb = Destino
x , y = Atual

D = Pr(Partida, Destino, Atual)

print(D)

xt = [x,xa,xb]
yt = [y,ya,yb]

plt.plot(xt,yt, 'ro')
plt.show
