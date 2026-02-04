#!/bin/bash
# Network QoS to prioritize robot motor control UDP over RDP TCP
# Run with sudo

IFACE="enp2s0"

echo "Setting up QoS on $IFACE..."

# Remove existing qdisc
tc qdisc del dev $IFACE root 2>/dev/null

# Create HTB root qdisc (allows bandwidth allocation)
tc qdisc add dev $IFACE root handle 1: htb default 30

# Root class - total bandwidth (1Gbps for gigabit ethernet)
tc class add dev $IFACE parent 1: classid 1:1 htb rate 1000mbit burst 15k

# HIGH PRIORITY: UDP traffic (motor control) - guaranteed 800Mbps, can burst to 1000
tc class add dev $IFACE parent 1:1 classid 1:10 htb rate 800mbit ceil 1000mbit burst 15k prio 1

# LOW PRIORITY: TCP traffic (RDP, SSH, etc) - guaranteed 100Mbps, can burst to 500
tc class add dev $IFACE parent 1:1 classid 1:20 htb rate 100mbit ceil 500mbit burst 15k prio 2

# DEFAULT: Everything else
tc class add dev $IFACE parent 1:1 classid 1:30 htb rate 100mbit ceil 300mbit burst 15k prio 3

# Add SFQ (Stochastic Fair Queuing) to each class for fairness within class
tc qdisc add dev $IFACE parent 1:10 handle 10: sfq perturb 10
tc qdisc add dev $IFACE parent 1:20 handle 20: sfq perturb 10
tc qdisc add dev $IFACE parent 1:30 handle 30: sfq perturb 10

# Filter: UDP traffic goes to high priority class
tc filter add dev $IFACE protocol ip parent 1:0 prio 1 u32 match ip protocol 17 0xff flowid 1:10

# Filter: TCP traffic goes to low priority class
tc filter add dev $IFACE protocol ip parent 1:0 prio 2 u32 match ip protocol 6 0xff flowid 1:20

echo "QoS configured!"
echo ""
echo "Priority classes:"
echo "  1:10 (HIGH)  - UDP (motor control) - 800Mbps guaranteed"
echo "  1:20 (LOW)   - TCP (RDP/SSH)       - 100Mbps guaranteed, 500Mbps max"
echo "  1:30 (DEFAULT) - Other             - 100Mbps guaranteed"
echo ""
echo "To verify: tc -s class show dev $IFACE"
echo "To remove: sudo tc qdisc del dev $IFACE root"
