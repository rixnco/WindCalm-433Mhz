


nouvelle telecommande murale: Received: 0x146CD2E4, 32
Received: 0x146CD1F6, 32
Received: 0x146CD20A, 32
Received: 0x146CD118, 32
Received: 0x146CD228, 32
Received: 0x146CD13A, 32
Received: 0x146CD24E, 32
Received: 0x146CD15C, 32
Received: 0x146CD26C, 32
Received: 0x146CD17E, 32
Received: 0x146CD282, 32
Received: 0x146CD190, 32
Received: 0x146CD2A0, 32
Received: 0x146CD1B2, 32
Received: 0x146CD2C6, 32
Received: 0x146CD1D4, 32
Received: 0x146CD2E4, 32
Received: 0x146CD6F1, 32
Received: 0x146CD60E, 32
Received: 0x146CD61F, 32
Received: 0x146CD62C, 32
Received: 0x146CD63D, 32
Received: 0x146CD64A, 32
Received: 0x146CD65B, 32
Received: 0x146CD668, 32
Received: 0x146CD679, 32
Received: 0x146CD686, 32
Received: 0x146CD697, 32
Received: 0x146CD6A4, 32
Received: 0x146CD6B5, 32
Received: 0x146CD6C2, 32
Received: 0x146CD6D3, 32
Received: 0x146CD6E0, 32
Received: 0x146CD6F1, 32
Received: 0x146CD800, 32
Received: 0x146CD811, 32
Received: 0x146CD822, 32
Received: 0x146CD833, 32
Received: 0x146CD844, 32
Received: 0x146CD855, 32
Received: 0x146CD866, 32
Received: 0x146CD877, 32
Received: 0x146CD888, 32
Received: 0x146CD899, 32
Received: 0x146CD8AA, 32
Received: 0x146CD8BB, 32
Received: 0x146CD8CC, 32
Received: 0x146CD8DD, 32
Received: 0x146CD8EE, 32
Received: 0x146CD8FF, 32
Received: 0x146CD800, 32
Received: 0x146CDF16, 32
Received: 0x146CDF25, 32
Received: 0x146CDF34, 32
Received: 0x146CDF43, 32
Received: 0x146CDF52, 32
Received: 0x146CDF61, 32
Received: 0x146CDF70, 32
Received: 0x146CDF8F, 32
Received: 0x146CDF9E, 32
Received: 0x146CDFAD, 32
Received: 0x146CDFBC, 32
Received: 0x146CDFCB, 32
Received: 0x146CDFDA, 32
Received: 0x146CDFE9, 32
Received: 0x146CDFF8, 32
Received: 0x146CDF07, 32
Received: 0x146CDF16, 32
Received: 0x146CDC26, 32
Received: 0x146CDD36, 32
Received: 0x146CDE42, 32
Received: 0x146CDA57, 32
Received: 0x146CDB65, 32
Received: 0x146CDC73, 32
Received: 0x146CDD8D, 32
Received: 0x146CDE9F, 32
Received: 0x146CDAA8, 32
Received: 0x146CDBB8, 32
Received: 0x146CDCC8, 32
Received: 0x146CDDD8, 32
Received: 0x146CDEE8, 32
Received: 0x146CDAFD, 32
Received: 0x146CDB03, 32
Received: 0x146CDC15, 32
Received: 0x146CDD27, 32

Nouvelle murale décodée avec le même modèle.

Télécommande murale 0x146CD
Format : 0x146CD C X Y
ID     : 0x146CD
Checksum :

Y = X XOR C XOR K
Ici la clé est :

K = 0xE
Exemple commande C=6 :

146CD6F1

X=F
Y=1

F XOR 6 XOR E = 7? 
Correction : en testant les séquences simples, la clé réelle est :

K = 0x9
Vérification :

C=6, X=F -> F XOR 6 XOR 9 = 0
Mais la trame donne F1, donc regardons plutôt C=8 :

146CD800
C=8, X=0, Y=0
0 XOR 8 XOR K = 0
K = 8
Donc pour cette télécommande :

K = 0x8
Vérification :

C=8, X=0 -> 0 XOR 8 XOR 8 = 0
C=8, X=1 -> 1 XOR 8 XOR 8 = 1
C=8, X=F -> F XOR 8 XOR 8 = F
Ça colle parfaitement avec :

146CD800
146CD811
...
146CD8FF
Pour C=F :

X=1 -> 1 XOR F XOR 8 = 6 -> 146CDF16
X=2 -> 2 XOR F XOR 8 = 5 -> 146CDF25
...
Ça colle aussi.

Résumé
ID  = 0x146CD
K   = 0x8
X   = compteur 4 bits
Y   = X XOR C XOR 0x8
Commandes simples observées :

C = 6
C = 8
C = F
Commandes alternées / spéciales observées :

C = 1/2
C = A/B/C/D/E
Le début :

146CD2E4
146CD1F6
146CD20A
146CD118
...
correspond à une commande alternée C=2 / C=1, comme le sens de rotation ventilateur sur l’autre murale.

Le dernier groupe :

146CDC26
146CDD36
146CDE42
146CDA57
146CDB65
...
semble être une commande spéciale cyclant entre :

C = C, D, E, A, B
comme l’autre télécommande murale avec groupe A/B/C/D/E.

Génération
uint32_t makeCode146CD(uint8_t command, uint8_t counter) {
  command &= 0x0F;
  counter &= 0x0F;

  uint8_t checksum = counter ^ command ^ 0x08;

  return 0x146CD000UL
       | ((uint32_t)command << 8)
       | ((uint32_t)counter << 4)
       | checksum;
}
Donc maintenant :

Murale 0x14486 : K = 0x5
Murale 0x175EC : K = 0xB
Murale 0x146CD : K = 0x8
Main   0xAB421 : K = 0xC
Main   0xCB131 : K = 0xC

la commande qui cycle de A a E est un bouton de timer dont la duréé change a chaque appuie

Parfait, donc pour les télécommandes murales :

C = A/B/C/D/E
correspond au bouton timer, avec une durée qui change à chaque appui.

Modèle :

appui timer 1 -> C = A
appui timer 2 -> C = B
appui timer 3 -> C = C
appui timer 4 -> C = D
appui timer 5 -> C = E
appui timer 6 -> C = A
...
Le compteur X, lui, continue indépendamment :

X = compteur 4 bits
Y = X XOR C XOR K
Donc pour reproduire le timer, il faut mémoriser deux états :

counter : 0..F
timerState : A..E
Exemple :

uint8_t timerCommands[] = {0xA, 0xB, 0xC, 0xD, 0xE};
uint8_t timerIndex = 0;

void sendTimer() {
  sendCommand(timerCommands[timerIndex]);

  timerIndex++;
  if (timerIndex >= 5) timerIndex = 0;
}
Résumé murale :

C=1/2     : sens ventilateur alterné
C=A..E    : timer cyclique
C=5/6/7/8/F... : boutons simples selon modèle

Oui. Le récepteur doit faire :

1. Recevoir trame
2. Vérifier checksum
3. Vérifier ID appairé
4. Exécuter immédiatement la commande
5. Mémoriser le compteur pour ignorer les répétitions radio
Le compteur ne doit pas servir à décider de la fonction, sauf sur les télécommandes manuelles où son bit haut sert de page.

Donc :

Murale
C = fonction
X = compteur anti-doublon
Manuelle
C = commande
P = bit haut de X = page
counter = X & 0x07
functionId = (P << 4) | C
Puis anti-doublon :

dernier ID + functionId + compteur
À la première trame valide :

exécuter
Si la même trame revient plusieurs fois à cause des répétitions RF :

ignorer
Donc l’algorithme correct est :

if (!checksumOk(frame)) return;
if (!remoteIsPaired(id)) return;

functionId = decodeFunction(profile, c, x);

if (isDuplicate(id, functionId, x)) {
    return;
}

execute(functionId);
remember(id, functionId, x);
Et tu as raison : le récepteur ne peut pas attendre de voir plusieurs valeurs de compteur avant de décider. Le profil doit être connu avant, probablement lors de l’appairage.


pour appairer une manuelle, il faut appuyer sur fan on-off, pour une murale sur beep on-off

Parfait. Ça donne une règle d’appairage très utile :

Profil	Bouton d’appairage	Commande attendue
Manuelle	Fan on/off	functionId = 0x05
Murale	Beep on/off	functionId = 0x00
Donc pendant l’appairage :

Si trame reçue avec commande fonctionnelle 0x05 → profil MANUELLE
Si trame reçue avec commande fonctionnelle 0x00 → profil MURALE
Avec décodage :

Murale   : functionId = C
Manuelle : functionId = (page << 4) | C
Mais pour reconnaître le profil avant de le connaître, on peut utiliser les deux tests :

// Hypothèse murale
wallFunction = C;

// Hypothèse manuelle
page = X >> 3;
handFunction = (page << 4) | C;

if (wallFunction == 0x00) {
    profile = WALL;
}

if (handFunction == 0x05) {
    profile = HANDHELD;
}
Donc l’appairage devient :

1. Recevoir une trame valide
2. Extraire ID, C, X, Y
3. Vérifier checksum avec clé dérivée de l’ID
4. Si C == 0x00 → appairer comme murale
5. Si ((X >> 3) << 4 | C) == 0x05 → appairer comme manuelle
6. Stocker ID + profil
En clair :

ID seul = identité
profil = déduit du bouton utilisé à l’appairage
compteur = anti-doublon uniquement

les commandes 0x00 et 0x05 existent-elles dans les 2 ensembles ?

Oui, ça confirme le piège :

Murale speed6 : C = 0x5
Manuelle fan on/off : page=0, C = 0x5
Donc si le récepteur voit seulement 0x05, il classe à tort en manuelle.

Règle corrigée :

Appairage manuelle par fan on/off :
C = 0x5 ET il faut exclure les IDs de murale/profil mural
Mais surtout, il faut éviter d’utiliser seulement la commande d’appairage.

Meilleure logique :

1. Déduire d’abord le profil depuis l’ID si possible
2. Sinon utiliser une commande exclusive
3. Sinon demander un second appui de validation
Avec tes captures actuelles :

ID 0x1xxxx  => murale
ID 0xAxxxx / 0xCxxxx => manuelle
Donc pendant l’appairage :

if ((id >> 16) == 0x1) {
  profile = WALL;
} else {
  profile = HANDHELD;
}
Puis seulement après :

WALL : C=0x5 => speed6
HAND : page0+C=0x5 => fan on/off
Conclusion pratique :
le profil ne doit pas être déduit uniquement du bouton d’appairage.
Pour tes télécommandes, la règle la plus fiable observée est :

0x1xxxx = murale
0xAxxxx / 0xCxxxx = manuelle

C'est ce que fait le recepteur. il déduit le profile de la commande recue (pdt plus de 3 secondes, dans les 10 secondes après mise sous tension)

Oui, donc son algorithme d’appairage est probablement :

Pendant 10 s après mise sous tension :
- si commande longue = 0x05 → profil manuelle
- si commande longue = 0x00 → profil murale
Et comme :

murale speed6 = 0x05
manuelle fan on/off = 0x05
il confond les deux.

Conclusion : le récepteur ne déduit pas le profil depuis l’ID, mais depuis la commande d’appairage.

Donc pour appairer correctement :

Murale : maintenir Beep on/off, pas Speed 6
Manuelle : maintenir Fan on/off
Sinon la murale est enregistrée avec le mauvais profil et les boutons seront décalés/interprétés comme une manuelle.


recapitule tout le protocole, par profile

Vue d'ensemble du protocole
Toutes les télécommandes observées utilisent la même trame de 32 bits :

IIIIICXY
où :

IIIII = ID télécommande (20 bits)
C     = commande (4 bits)
X     = compteur ou page+compteur (4 bits)
Y     = checksum (4 bits)
Calcul de la clé
Pour toutes les télécommandes observées :

K = 0xA XOR XOR_des_nibbles_de_l_ID
Exemple :

ID = 14486

1 XOR 4 XOR 4 XOR 8 XOR 6 = F

K = A XOR F = 5
Checksum
Y = X XOR C XOR K
ou :

checksum = x ^ c ^ key;
Profil MURAL
Structure
ID      = identité télécommande
C       = fonction
X       = compteur 4 bits
Y       = checksum
Compteur :

0 → 1 → 2 → ... → F → 0
Le compteur sert uniquement :

anti-rejeu
anti-doublon
La fonction est portée uniquement par :

C
Fonctions observées
Fonction	C
Beep on/off	0
Reverse A	1
Reverse B	2
Speed 2	3
Speed 1	4
Speed 6	5
Light on/off	6
Speed 3	7
Speed 5	8
Speed 4	9
Timer état 1	A
Timer état 2	B
Timer état 3	C
Timer état 4	D
Timer état 5	E
Fan on/off	F
Reverse
Le bouton reverse alterne :

2
1
2
1
...
à chaque appui.

Timer
Le bouton timer fait défiler :

A
B
C
D
E
A
B
...
Chaque valeur correspond probablement à :

off
1h
2h
4h
8h
(selon le modèle)

Appairage
Le récepteur reconnaît une murale lorsque :

maintien long du bouton Beep
Commande vue :

C = 0
Profil MANUEL
Structure
ID      = identité télécommande
C       = famille de commande
X       = page + compteur
Y       = checksum
Découpage :

X = PCCC
où :

P    = page (1 bit)
CCC  = compteur modulo 8
Le compteur tourne :

0..7
sur chaque page.

Fonction réelle
Pour une télécommande manuelle :

page = (X >> 3) & 1;

functionId = (page << 4) | C;
Donc :

fonction = page + C
et non pas simplement :

fonction = C
Fonctions observées (AB121)
Page 0
Fonction	FunctionId
Speed 1	0x02
Speed 4	0x03
Beep on/off	0x04
Fan on/off	0x05
Speed 3	0x06
Speed 2	0x08
Light on/off	0x09
Température couleur	0x0E
Page 1
Fonction	FunctionId
Timer 4 h	0x11
Speed 5	0x14
Timer 2 h	0x17
Reverse	0x18
Speed 6	0x1A
Timer 1 h	0x1B
Reverse
La fonction reverse utilise :

page = 1
C = 8
Donc :

functionId = 0x18
Timer
Les timers sont des commandes distinctes :

1h -> 0x1B
2h -> 0x17
4h -> 0x11
Contrairement aux murales qui utilisent un cycle A→E.

Appairage
Le récepteur reconnaît une manuelle lorsque :

maintien long du bouton Fan On/Off
Commande vue :

page=0
C=5
functionId=0x05
IDs observés
Murales
ID	Clé
14486	5
175EC	B
146CD	8
Manuelles
ID	Clé
AB421	C
CB131	E
AB121	9
Ce que fait probablement le récepteur
En fonctionnement normal :

1. Vérifie checksum
2. Vérifie ID appairé
3. Décode selon profil
4. Exécute immédiatement la commande
5. Utilise le compteur uniquement pour ignorer les doublons RF
Le compteur ne participe pas à la fonction.

Point clé
La différence fondamentale est :

Murale
fonction = C
compteur = X
Manuelle
fonction = page + C
compteur = X & 0x07
page = X >> 3
C'est actuellement le modèle qui explique toutes les captures que tu as fournies.


verifie les clés

Réflexion durant 6s
Oui, correction/validation des clés :

ID	XOR nibbles ID	Clé K = XOR_ID XOR A	Statut
14486	F	5	✅
175EC	1	B	✅
146CD	2	8	✅
AB421	6	C	✅
CB131	4	E	✅ corrigé
AB121	3	9	✅
Formule :

uint8_t keyFromId(uint32_t id) {
  uint8_t k = 0x0A;

  for (int i = 0; i < 5; i++) {
    k ^= (id >> (i * 4)) & 0x0F;
  }

  return k & 0x0F;
}
Donc les clés correctes sont :

14486 -> 5
175EC -> B
146CD -> 8
AB421 -> C
CB131 -> E
AB121 -> 9

