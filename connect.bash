#!/bin/bash

set -e

echo "v0.1.0"

# define config
mkdir -p "$HOME/.config/raf"
raf_credentials_file="$HOME/.config/raf/credentials.yaml"
mkdir -p "$HOME/.cache/raf"
raf_token_file="$HOME/.cache/raf/token.json"
raf_ovpn_file="$HOME/.config/raf/openvpn_config.ovpn"
api_host="https://raf.e-yantra.org"
# api_host="http://localhost:8000"

# ensure openvpn exists
if ! command -v openvpn &> /dev/null; then
  echo "'openvpn' could not be found. Please install openvpn before proceding using the following commands:
  sudo apt update
  sudo apt install openvpn"
  exit 1
fi

# ensure jq exists
if ! command -v jq &> /dev/null; then
  echo "'jq' could not be found. Please install jq before proceding using the following commands:
  sudo apt update
  sudo apt install jq"
  exit 1
fi

# ensure credentials file exists, else show sample of credentials file
if [ ! -f "$raf_credentials_file" ]; then
  echo "Credentials file doesn't exist."
  echo "username: your username goes here" > "$raf_credentials_file"
  echo "password: your password goes here" >> "$raf_credentials_file"
  echo "A sample file has been created for you at '$raf_credentials_file'. Modify it add your username and password."
  exit 1
fi

# fetch email and password from local config file
username=$(sed -n -e 's/^username: //p' "$raf_credentials_file")
password=$(sed -n -e 's/^password: //p' "$raf_credentials_file")

# fetch access token and refresh token from raf
curl -L --silent --request POST --url "$api_host/auth/jwt/create" --header 'Content-Type: application/json' --data "{\"username\": \"$username\", \"password\": \"$password\"}" | jq > "$raf_token_file"

# ensure curl didnt return failure response
detail=$(jq -r .detail "$raf_token_file")
if [ ! "$detail" = "null" ]; then
  echo "Unable to authorize you because of the following reason: $detail"
  echo "Please check your credentials in $raf_credentials_file"
  exit 1
fi

# store access token and refresh token got from raf to cache
access_token=$(jq -r .access "$raf_token_file")

# check if ovpn config exists, else download it
if [ ! -f "$raf_ovpn_file" ]; then
  curl -L --silent --request GET --url "$api_host/get-openvpn-config/" --header "Authorization: Bearer $access_token" --header 'Content-Type: application/json' | jq -r .config > "$raf_ovpn_file"
fi

# check if ovpn config was fetched correctly
if ! grep -q '[^[:space:]]' "$raf_ovpn_file"; then
  echo "Unable to fetch your OpenVPN config file. Please contact the administrators."
  rm "$raf_ovpn_file"
  exit 1
fi

# use access token to get openvpn username and password for vpn from raf
vpn_creds_json=$(curl -L --silent --request GET --url "$api_host/get-openvpn-creds/" --header "Authorization: Bearer $access_token" --header 'Content-Type: application/json' | jq)

# create temp file
temp_vpn_creds=$(mktemp)
success=$(echo "$vpn_creds_json" | jq -r .success)
if [ "$success" = "false" ]; then
  echo "$vpn_creds_json" | jq -r .message
  exit 1
fi

# put username and password in tempfile
echo "$vpn_creds_json" | jq -r .username > "$temp_vpn_creds"
echo "$vpn_creds_json" | jq -r .password >> "$temp_vpn_creds"

# verbosity
date
cat "$temp_vpn_creds"

# connect to vpn
sudo openvpn --config "$raf_ovpn_file" --auth-user-pass "$temp_vpn_creds"

# delete tempfile
rm -f "$temp_vpn_creds"
