# -*- coding: utf-8 -*-
"""
Created on Tue Jan  1 02:20:33 2019

@author: jucem
"""

import pygame

pygame.init()

pygame.display.set_mode((100, 100))

looping = True

while looping:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            looping = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                print('Forward')
            elif event.key == pygame.K_s:
                print('Backward')
            elif event.key == pygame.K_q:
                looping = False
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_LEFT:
                print('stand_left')
            if event.key == pygame.K_RIGHT:
                print('stand_right')
            if event.key == pygame.K_UP:
                print('stand_up')
            if event.key == pygame.K_DOWN:
                print('stand_down') 
pygame.quit()