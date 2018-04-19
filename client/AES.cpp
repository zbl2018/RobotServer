#include"./include/AES.h"
bool AES::initKV(char temp_key[],char temp_iv[])  
{  
    if(strlen(temp_key)!=CryptoPP::AES::DEFAULT_KEYLENGTH||strlen(temp_iv)!=CryptoPP::AES::BLOCKSIZE)
    {
        printf("the length of key or iv is error!\nthe length is 16.");
        return false;
    }

    // memset( AES::key, 0x00, CryptoPP::AES::DEFAULT_KEYLENGTH );  
    // memset( AES::iv, 0x00, CryptoPP::AES::BLOCKSIZE );       
    // char tmpK[] = key; 
    // char tmpIV[] =iv; 
    for (int j = 0; j < CryptoPP::AES::DEFAULT_KEYLENGTH; ++j) 
    { 
        AES::key[j] = temp_key[j]; 
    } 
 
    for (int i = 0; i < CryptoPP::AES::BLOCKSIZE; ++i) 
    { 
        AES::iv[i] = temp_key[i]; 
    } 

    return true;
}    
string AES::encrypt(string plainText)  
{  
    string cipherText;    
    CryptoPP::AES::Encryption aesEncryption(key, CryptoPP::AES::DEFAULT_KEYLENGTH);  
    CryptoPP::CBC_Mode_ExternalCipher::Encryption cbcEncryption( aesEncryption, iv );  
    CryptoPP::StreamTransformationFilter stfEncryptor(cbcEncryption, new CryptoPP::StringSink( cipherText ));  
    stfEncryptor.Put( reinterpret_cast<const unsigned char*>( plainText.c_str() ), plainText.length() + 1 );  
    stfEncryptor.MessageEnd();  
    string cipherTextHex;  
    for( int i = 0; i < cipherText.size(); i++ )  
    {  
        char ch[3] = {0};  
        sprintf(ch, "%02x",  static_cast<byte>(cipherText[i]));  
        cipherTextHex += ch;  
    }   
    return cipherTextHex;  
}  

string AES::decrypt(string cipherTextHex)  
{  
    string cipherText;  
    string decryptedText;  
  
    int i = 0;  
    while(true)  
    {  
        char c;  
        int x;  
        stringstream ss;  
        ss<<hex<<cipherTextHex.substr(i, 2).c_str();  
        ss>>x;  
        c = (char)x;  
        cipherText += c;  
        if(i >= cipherTextHex.length() - 2)break;  
        i += 2;  
    }  
    CryptoPP::AES::Decryption aesDecryption(key, CryptoPP::AES::DEFAULT_KEYLENGTH);  
    CryptoPP::CBC_Mode_ExternalCipher::Decryption cbcDecryption( aesDecryption, iv );  
    try{
            CryptoPP::StreamTransformationFilter stfDecryptor(cbcDecryption, new CryptoPP::StringSink( decryptedText ));  
            stfDecryptor.Put( reinterpret_cast<const unsigned char*>( cipherText.c_str() ), cipherText.size());  
            stfDecryptor.MessageEnd(); 
    }
    catch(...){
        printf("decrypt error!\n");
        return DECRYPTION_ERROR;
    }
     
    return decryptedText;  
} 
  
// void writeCipher(string output)  
// {  
//     ofstream out("/tmp/cipher.data");  
//     out.write(output.c_str(), output.length());  
//     out.close();  
  
//     cout<<"writeCipher finish "<<endl<<endl;  
// }  
  
  
  
// int main()  
// {  
//     // static AES::key="1234567890123456";
//     // static AES::key=
//     AES my_aes;
//     string text = "hello zbl !";  
//     cout<<"text : "<<text<<endl;  
//     my_aes.initKV("1234567890123456","1234567890123456");  
//     string cipherHex =my_aes.encrypt(text);  
//     cout<<"cipher : "<<cipherHex<<endl;  
//     //writeCipher(cipherHex); 
//      my_aes.initKV("1004567890123456","1234567890123456");
//     cipherHex =my_aes.decrypt(cipherHex);
//     cout<<"cipher : "<<cipherHex<<endl; 
//     return 0;  
// } 