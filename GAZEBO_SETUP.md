# Gazebo GUI Setup - Docker Solution

## Step 1: Start All Containers
In PowerShell (in project directory):
```bash
docker-compose -f docker-compose.dev.yml up -d --build
```

## Step 2: Run Gazebo
In PowerShell (any directory):
```bash
docker exec -it ros2_humble_gazebo_harmonic ign gazebo
```

## Step 3: View Gazebo GUI
Open web browser and go to:
```
http://localhost:8080
```

**That's it!** The Docker X11 server handles everything automatically.

---

## Alternative: VcXsrv Method (if Docker method doesn't work)

1. **Install VcXsrv**: Download from https://sourceforge.net/projects/vcxsrv/
2. **Start VcXsrv**: Find "XLaunch" in Start Menu, check "Disable access control"
3. **Set display**: `$env:DISPLAY = "host.docker.internal:0.0"` in PowerShell
4. **Remove x11-server** from docker-compose.dev.yml
5. **Change DISPLAY** in ros2-humble-gazebo to `${DISPLAY}`
6. **Restart containers**