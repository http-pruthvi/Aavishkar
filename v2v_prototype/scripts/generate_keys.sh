#!/bin/bash
openssl ecparam -name prime256v1 -genkey -noout -out private_key.pem
openssl ec -in private_key.pem -pubout -out public_key.pem
echo "Keys generated: private_key.pem, public_key.pem"
