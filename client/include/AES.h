#include <stdio.h>    
#include <iostream>  
#include <fstream>  
#include <sstream>
#include "../include/cryptopp/aes.h"  
#include "../include/cryptopp/filters.h"  
#include "../include/cryptopp/modes.h"  
  
using namespace std;  
#define DECRYPTION_ERROR "crypetion error"
#define byte uint8_t 
class AES{
    public:
            bool initKV(char temp_key[],char temp_iv[]);
            string encrypt(string plainText);
            string decrypt(string cipherTextHex);
    private:
            byte key[ CryptoPP::AES::DEFAULT_KEYLENGTH ], iv[ CryptoPP::AES::BLOCKSIZE];       
};
