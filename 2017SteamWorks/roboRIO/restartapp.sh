echo Stopping CheezDroid...
adb shell am force-stop com.team254.cheezdroid
while adb shell ps | grep -q team254; do  sleep 1; done
echo \[done]\\n

echo Starting CheezDroid...
adb shell monkey -p com.team254.cheezdroid 1
sleep 2
if adb shell ps | grep -q team254
 then echo OK
 else
  sleep 2
  if adb shell ps | grep -q team254
   then echo OK
   else
    sleep 2
    if adb shell ps | grep -q team254
     then echo OK
    fi
  fi
fi

# basically, we've got <del>three</del>six seconds to start the app, or else we don't get the happy OK message :(
# also, d2u