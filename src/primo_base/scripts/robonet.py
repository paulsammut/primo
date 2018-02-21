#!/usr/bin/python

import sys
import argparse
from tempfile import mkstemp
from os import fdopen, remove, chmod, chown
from shutil import move
import os
import socket
import pwd
import grp
import subprocess

robot_name = "primo"
laptop_name = "dev-laptop"
hosts_fp = "/etc/hosts"              # hosts file path
zshrc_fp = os.path.expanduser('~/.zshrc')

# Create the dictionaries for the ip addresses
robot_ip = {
        "shop" : "192.168.1.104",
        "VPN"  : "10.8.0.1",
        "field": "192.168.0.100"
        }

laptop_ip ={
        "shop" : "192.168.1.142",
        "VPN"  : "10.8.0.2",
        "field": "192.168.0.101"
        }

def configHosts(remote_host_name, remote_host_ip):
    # create a temp file
    fh, abs_path = mkstemp()
    temp_file = fdopen(fh,'w')

    # read the hosts file
    hosts_file = open(hosts_fp,"r").read()

    # grab the file and split it into a list of lines
    line_list = hosts_file.split("\n")

    # Iterate through the list of lines. We use enumerate to be able to modify
    # the element in the list
    for idx, line in enumerate(line_list):
        # Find the ROS_MASTER entry
        if (remote_host_name in line) and not "#" in line:
            # Replace the entry with one network and remote host name
            line_list[idx] = remote_host_ip + " " + remote_host_name
    
    # Join the lines together with \n and create our text files
    temp_file.write('\n'.join(line_list))

    # remove the original file
    remove(hosts_fp)

    # move the modified hosts file to replace it
    move(abs_path, hosts_fp)

    # change the permissions
    chmod(hosts_fp, 0644)

    print "Set " + remote_host_name + " with ip " + remote_host_ip 

def configROSMASTER(master_host_name):
    # create a temp file
    fh, abs_path = mkstemp()
    temp_file = fdopen(fh,'w')

    # read the current .zshrc file
    zshrc_file = open(zshrc_fp,"r").read()

    new_uri = "http://"+master_host_name+":11311";

    # grab the file and split it into a list of lines
    line_list = zshrc_file.split("\n")

    # Iterate through the list of lines. We use enumerate to be able to modify
    # the element in the list
    for idx, line in enumerate(line_list):
        # Find the ROS_MASTER entry
        if ("export ROS_MASTER_URI" in line) and (not "#" in line):
            # Replace the entry with the right URI
            line_list[idx] = "export ROS_MASTER_URI="+new_uri
    
    # Join the lines together with \n and create our text files
    temp_file.write('\n'.join(line_list))

    # remove the original file
    remove(zshrc_fp)

    # move the modified hosts file to replace it
    move(abs_path, zshrc_fp)

    # change the permissions
    chmod(zshrc_fp, 0644)
    uid = pwd.getpwnam("paul").pw_uid
    gid = grp.getgrnam("paul").gr_gid
    chown(zshrc_fp, uid, gid)

    # export the ros master. COMMENT: THIS DOESN"T WORK!
    # os.system("export ROS_MASTER_URI="+new_uri)

    print "Set the ROS_MASTER_URI to: " + new_uri

if __name__ == "__main__":

    # Make sure we are in sudo
    if not os.geteuid() == 0:
        sys.exit('Script must be run as root')

    # Handle the argument parsing
    parser = argparse.ArgumentParser(description='Configure the network settings for our laptop and robot')

    parser.add_argument("-n","--network", help="network the devices are on",
                                    choices=('VPN', 'shop', 'field'))

    parser.add_argument("-r","--rosmaster", help="The ros master hostname",
                                    choices=(laptop_name, robot_name))

    args = parser.parse_args()

    # Handle the dev-laptop host
    # Here we open the hosts file and change the primo IP to the appropriate one.
    if socket.gethostname() == laptop_name:
        # Make a check to make sure the network arguments have been added
        if args.network == None:
            print "Network must be specified!"
            quit()

        configHosts(robot_name, robot_ip.get(args.network))

    if socket.gethostname() == robot_name:
        # Make a check to make sure the network arguments have been added
        if args.network == None:
            print "Network must be specified!"
            quit()
        
        configHosts(laptop_name, laptop_ip.get(args.network))

    # Handle the ros master
    if args.rosmaster:
        configROSMASTER(args.rosmaster)
