# Etilotest

Student: Dajka Zsolt
Grupa: 4765 SECI

Proiect de SODTR : 
- utlizand o placa STM32F429-DISCO
- crearea unei interfete grafice 
- citirea valorii unui senzor

Probleme intampinate/solutii:
- Embedded Wizzard genereaza nu doar fisiere sursa ci si librarii. Pentru a putea compila librariile aceste trebuie incluse in Atollic studio in sectiunea Path and symbols 
- Librariile generate sunt de forma libewrte.a. Pentru a putea include aceste librarii in "Library Path" trebuie precizata calea pe cand in Libraries numele fisierului dar fara extensie si fara prefixul lib.
- In Build setting sectiunea Target, obtinea de Floating point trebue setata pe Hardware implementation ca compilarea sa fie identica cu librariile generate de EW.
- Calea catre fisierele sursa si librarii trebuie sa fie precizate pentru configuratia Debug cat si pentru Release.

Deoarece senzorul avut la dispozitie este unul de gaz s-a renuntat la senzorul de temperatura.

Fisierul etilotesc contine fisierele sursa pentru aplicatie.


