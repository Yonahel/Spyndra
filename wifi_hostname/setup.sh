#!/bin/bash
# install avahi-daemon if necessary
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' avahi-daemon|grep "install ok installed")
echo Checking for avahi-daemon: $PKG_OK
if [ "" == "$PKG_OK" ]; then
  echo "No somelib. Setting up somelib."
  sudo apt-get --force-yes --yes install avahi-daemon
fi

# Add configuration file
if [ -f "multiple.service" ]; then
  sudo cp multiple.service /etc/avahi/services/multiple.service
else
  echo 'File multiple.service does not exist.'
fi

# There are many raspberry pi's in Creative Machines Lab.
# In order to disambiguate, change hastname to "spyndra"
OLD_HOSTNAME=$(hostname)
NEW_HOSTNAME="raspberrypi"
sudo echo $NEW_HOSTNAME > /etc/hostname
# After change, restart avahi to activate the change
#sudo perl -pi -e "s/""$OLD_HOSTNAME""$NEW_HOSTNAME"/g /etc/hosts
sudo perl -pi -e "s/$OLD_HOSTNAME/$NEW_HOSTNAME/g" /etc/hosts
sudo /etc/init.d/avahi-daemon restart

# ping spyndra.local to validate
ping "$NEW_HOSTNAME".local
