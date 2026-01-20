# Setup SSH Key Authentication (no password required)

## Problem
The SSH tunnel requires password entry every time.

## Solution: Use SSH Keys

With SSH keys you can connect **without password**!

---

## 🔑 Step-by-Step Guide

### **1. Generate SSH Key on Windows**

**Open PowerShell** and run:

```powershell
ssh-keygen -t rsa -b 4096 -C "your_pc@windows"
```

**Answer the prompts:**
- **Save location:** Just press `Enter` (default: `C:\Users\SCMJ178\.ssh\id_rsa`)
- **Passphrase:** Just press `Enter` (no password) or enter an optional password

**Output:**
```
Your identification has been saved in C:\Users\SCMJ178\.ssh\id_rsa
Your public key has been saved in C:\Users\SCMJ178\.ssh\id_rsa.pub
```

---

### **2. Copy Public Key to Raspberry Pi**

**PowerShell:**

```powershell
type C:\Users\SCMJ178\.ssh\id_rsa.pub | ssh pi@192.168.2.126 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys && chmod 700 ~/.ssh"
```

**You need to enter the password ONE MORE TIME.**

---

### **3. Test**

**PowerShell:**

```powershell
ssh pi@192.168.2.126
```

**Expected result:** You will **NOT** be asked for a password anymore! ✅

---

### **4. Use Batch File**

Now you can use the batch file **without password entry**:

```bat
start_ssh_tunnel.bat
```

The SSH tunnel starts automatically! 🎉

---

## 🎯 Benefits

✅ **No password required anymore**  
✅ **Faster connection**  
✅ **More secure** (longer key than password)  
✅ **Automation possible**

---

## 🔧 Alternative: Store password in batch (NOT recommended!)

**⚠️ WARNING: Insecure! Password in plain text!**

If you don't want to use SSH keys, you can use `sshpass`:

### **1. Install sshpass for Windows**

Download from: https://github.com/PowerShell/Win32-OpenSSH/releases

### **2. Modify Batch File**

```bat
@echo off
echo Starting SSH Tunnel...
sshpass -p "YOUR_PASSWORD_HERE" ssh -L 10223:localhost:10223 -L 5555:localhost:5555 pi@192.168.2.126
```

**⚠️ Disadvantage:** Anyone who opens the file sees your password!

---

## 📋 Recommendation

**Use SSH Keys (Solution 1)** – secure and convenient! 🔑

---

## Troubleshooting

### **Problem: "Permission denied (publickey)"**

**Check on Raspberry Pi:**

```bash
ls -la ~/.ssh/
```

**Expected output:**
```
-rw------- 1 pi pi  authorized_keys
drwx------ 2 pi pi  .ssh
```

**If permissions are wrong:**

```bash
chmod 700 ~/.ssh
chmod 600 ~/.ssh/authorized_keys
```

---

### **Problem: SSH still asks for password**

**Check if key is used:**

```powershell
ssh -v pi@192.168.2.126
```

**Look for:**
```
debug1: Offering public key: C:/Users/SCMJ178/.ssh/id_rsa RSA SHA256:...
debug1: Server accepts key: ...
```

If this doesn't appear, the key was not copied correctly.

---

## ✅ After successful setup

The batch file `start_ssh_tunnel.bat` starts the tunnel **automatically without password prompt**!
