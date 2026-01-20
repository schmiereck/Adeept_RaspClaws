# SSH-Key-Authentifizierung einrichten (kein Passwort mehr nötig)

## Problem
Beim SSH-Tunnel muss jedes Mal das Passwort eingegeben werden.

## Lösung: SSH-Keys verwenden

Mit SSH-Keys kannst du dich **ohne Passwort** verbinden!

---

## 🔑 Schritt-für-Schritt-Anleitung

### **1. SSH-Key auf Windows generieren**

**PowerShell öffnen** und ausführen:

```powershell
ssh-keygen -t rsa -b 4096 -C "dein_pc@windows"
```

**Fragen beantworten:**
- **Speicherort:** Einfach `Enter` drücken (Standard: `C:\Users\SCMJ178\.ssh\id_rsa`)
- **Passphrase:** Einfach `Enter` drücken (kein Passwort) oder ein optionales Passwort eingeben

**Ausgabe:**
```
Your identification has been saved in C:\Users\SCMJ178\.ssh\id_rsa
Your public key has been saved in C:\Users\SCMJ178\.ssh\id_rsa.pub
```

---

### **2. Public Key auf Raspberry Pi kopieren**

**PowerShell:**

```powershell
type C:\Users\SCMJ178\.ssh\id_rsa.pub | ssh pi@192.168.2.126 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys && chmod 700 ~/.ssh"
```

**Du musst jetzt noch EINMAL das Passwort eingeben.**

---

### **3. Testen**

**PowerShell:**

```powershell
ssh pi@192.168.2.126
```

**Erwartetes Ergebnis:** Du wirst **NICHT** mehr nach dem Passwort gefragt! ✅

---

### **4. Batch-Datei verwenden**

Jetzt kannst du die Batch-Datei verwenden **ohne Passwort-Eingabe**:

```bat
start_ssh_tunnel.bat
```

Der SSH-Tunnel startet automatisch! 🎉

---

## 🎯 Vorteile

✅ **Kein Passwort mehr nötig**  
✅ **Schnellere Verbindung**  
✅ **Sicherer** (längerer Key als Passwort)  
✅ **Automatisierung möglich**

---

## 🔧 Alternative: Passwort in Batch speichern (NICHT empfohlen!)

**⚠️ WARNUNG: Unsicher! Passwort liegt im Klartext!**

Falls du SSH-Keys nicht verwenden willst, kannst du `sshpass` verwenden:

### **1. sshpass für Windows installieren**

Download von: https://github.com/PowerShell/Win32-OpenSSH/releases

### **2. Batch-Datei anpassen**

```bat
@echo off
echo Starting SSH Tunnel...
sshpass -p "DEIN_PASSWORT_HIER" ssh -L 10223:localhost:10223 -L 5555:localhost:5555 pi@192.168.2.126
```

**⚠️ Nachteil:** Jeder, der die Datei öffnet, sieht dein Passwort!

---

## 📋 Empfehlung

**Verwende SSH-Keys (Lösung 1)** – sicher und komfortabel! 🔑

---

## Troubleshooting

### **Problem: "Permission denied (publickey)"**

**Auf dem Raspberry Pi prüfen:**

```bash
ls -la ~/.ssh/
```

**Erwartete Ausgabe:**
```
-rw------- 1 pi pi  authorized_keys
drwx------ 2 pi pi  .ssh
```

**Falls Rechte falsch sind:**

```bash
chmod 700 ~/.ssh
chmod 600 ~/.ssh/authorized_keys
```

---

### **Problem: SSH fragt immer noch nach Passwort**

**Prüfe, ob der Key verwendet wird:**

```powershell
ssh -v pi@192.168.2.126
```

**Suche nach:**
```
debug1: Offering public key: C:/Users/SCMJ178/.ssh/id_rsa RSA SHA256:...
debug1: Server accepts key: ...
```

Falls das nicht erscheint, wurde der Key nicht korrekt kopiert.

---

## ✅ Nach erfolgreicher Einrichtung

Die Batch-Datei `start_ssh_tunnel.bat` startet den Tunnel **automatisch ohne Passwort-Abfrage**!
