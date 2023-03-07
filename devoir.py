import numpy as np
import math as math
import matplotlib.pyplot as plt

tau     =   2    # valeur taux compression [-]
D       =   2    # valeur alesage           [m]
C       =   3    # valeur course [m]
L       =   4    # valeur longueur bielle@ [m]
mpiston =   5   # valeur masse piston@ [kg]
mbielle =   6   # valeur masse bielle  [kg]
Q       =   1650   # valeur chaleur emise par fuel par kg de melange admis@ [J/kg_inlet gas]

###############################
#    Runge-Kutta (pour p)     #
###############################

def integrator(Xstart,Ustart,Xend,h,f):
    """
    h le pas entre chaque X
    f la fonction à intégrer
    """
    imax = int((Xend-Xstart)/h)
    X = Xstart + np.arange(imax+1)*h
    U = np.zeros((imax+1,3)); U[0,:] = Ustart
    for i in range(imax):
        K1 = f(U[i,:]       )
        K2 = f(U[i,:]+K1*h/2)
        K3 = f(U[i,:]+K2*h/2)
        K4 = f(U[i,:]+K3*h  )
        U[i+1,:] = U[i,:] + h*(K1+2*K2+2*K3+K4)/6     
    return X,U


def myfunc(rpm, s, theta, thetaC, deltaThetaC):
    """ 
    dimBielle dimensionnement d'une bielle
    dimBielle(rpm, s, theta, thetaC, deltaThetaC) calcules les données thermodynamiques
    et les forces d'un système Piston-Bielle-Vilebrequin afin de dimensionner la section
    d'une bielle.

    INPUTS :
    rpm : vitesse angulaire du moteur [rotation per minute]
    s : surcharge du moteur [-]
    theta : angle auxquels renvoyer les données [°]
    thetaC : angle d'allumage [°]
    deltaThetaC : durée de la combustion (en angle) [°] 
    theta : angle auxquels renvoyer les données [°]
    (Les angles sont donnés entre 0 et 720°)

    OUTPUTS :
    t : section de la bielle [m]
    V(theta) : Volume de la chambre de combustion en fonction de theta [m3]
    Q(theta) : chaleur dégagée par la combustion en fonction de theta [J]
    F_pied(theta) : [N]
    F_tete(theta) : [N]
    F_inertie(theta) : [N]
    (une force est positive si dirigée vers le haut).
    """
	
	
    """""""""""""""""
    Calcul du V_output en fonction de thêta
    
    """""""""""""""""
    
    V_output = np.zeros(theta.size)
    print(theta.size)
    print(V_output)
    R = C / 2 # Longueur de manivelle
    V_cylindree= (( math.pi * pow(D, 2)) / 4 ) * (2 * R)
    beta = L / R # rapport entre la longueur de la bielle et la longueur de la manivelle
    V_min = (1 / ( tau - 1)) * V_cylindree
    V_max = V_min * tau
    print(theta)
    print(V_min)
    print(V_max)
    print(V_cylindree)
    for i in range (len(theta)):
        V_output[i] = (V_cylindree / 2) * (1 - math.cos(theta[i]) + beta - math.sqrt(pow(beta, 2) - pow(math.sin(theta [i]), 2))) + V_min 
    
    """""""""""""""""
    Graphe de V_output en fct de theta
    
    """""""""""""""""
    markerline, stemlines, baseline = plt.stem(theta, V_output/V_max)
    
    # Axes
    fs_text = 16 # Taille du texte
    plt.xlabel("$Thêta$ [degré]", fontsize=fs_text)
    plt.ylabel("$V_output$ [m^3]", fontsize=fs_text)
    
    # Titre
    plt.title("Volume disponible pour le gaz en fonction de l'angle de vilebrequin", fontsize=fs_text)
    plt.show()
    

    """""""""""""""""
    Calcul du Q_output en fonction de thêta 

    """""""""""""""""
    poids_air = np.array(V_output * 1.292)
    Q_output = np.zeros(theta.size)
    Q_max = float('-inf')
    Q_min = float('inf')
    for i in range (len(theta)):
        arg = math.pi * ((theta[i] - thetaC) / deltaThetaC )
        Q_output[i] = ((Q * poids_air[i]) / 2) * (1 - math.cos(math.radians(arg)))
            
        if (Q_output[i] > Q_max) :
            Q_max = Q_output[i]
        if (Q_output[i] < Q_min) :
            Q_min = Q_output[i]
    
    """""""""""""""""
    Graphe de Q_output en fct de theta
    
    """""""""""""""""
    
    plt.plot(theta, Q_output)
    
    # Axes
    fs_text = 16 # Taille du texte
    plt.xlabel("$Thêta$ [degré]", fontsize=fs_text)
    plt.ylabel("$Q_{output}$ [J]", fontsize=fs_text)
    
    # Titre
    plt.title("Q_max = " + str(Q_max) + "    Q_min = " + str(Q_min))
    plt.suptitle("Chaleur en fonction de l'angle de vilebrequin", fontsize=fs_text)
    plt.show()
    """""""""""""""""
    Calcul du F_pied_output en fonction de thêta 

    """""""""""""""""
    
     
    """""""""""""""""
    Calcul du F_tete_output en fonction de thêta 

    """""""""""""""""
    
     
    """""""""""""""""
    Calcul du p_output en fonction de thêta 

    """""""""""""""""
    
     
    """""""""""""""""
    Calcul du t 

    """""""""""""""""
    
    
    Q_output      = 2*np.ones_like(theta);
    F_pied_output = 3*np.ones_like(theta);
    F_tete_output = 4*np.ones_like(theta);
    p_output      = 5*np.ones_like(theta);
    t             = 6;
    
    return V_output, Q_output, F_pied_output, F_tete_output, p_output, t  ;

