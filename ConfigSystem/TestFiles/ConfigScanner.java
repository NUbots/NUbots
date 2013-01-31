import java.io.*;
import org.json.*;

class ConfigScanner
{
    LineNumberReader lnr = null;

    public ConfigScanner(LineNumberReader _lnr)
    {
        lnr = _lnr;
    }
    
    private String currentToken = "";
    private Token rntLastToken = null;
    public Token getNextToken(LineNumberReader lnrH) throws IOException
    {
        Token t = peekNextToken(lnr);
        rntLastToken = null;
        return t;
    }
    public Token peekNextToken(LineNumberReader lnr) throws IOException
    {
        if(rntLastToken == null)
        {
            rntLastToken = readNextToken(lnr);
            return rntLastToken;
        }
        else return rntLastToken;
    }
    

    public void skipTokens(LineNumberReader lnr, int num) throws IOException
    {
        for(int i = 0; i < num; i++) getNextToken(lnr);
    }
    
    public Token getCheckedToken(LineNumberReader lnr, int type) throws IOException
    {
        // for(int i : args)
        // {
            Token t = getNextToken(lnr);
            if(t!= null && t.type == type) return t;
            
            System.out.println(
                "ERROR [line "+lnr.getLineNumber()+
                "]: Token '" + (t == null? "<null>" : t.toString()) +
                "' is not of expected type '" + (char)type + "'"
                );
            System.exit(1);
        // }

        return null;
    }
    public boolean checkTokens(LineNumberReader lnr, int... args) throws IOException
    {
        for(int i : args)
        {
            Token t = getNextToken(lnr);
            if(t!= null && t.type == i) continue;
            
            System.out.println(
                "ERROR [line "+lnr.getLineNumber()+
                "]: Token '" + (t == null? "<null>" : t.toString()) +
                "' is not of expected type '" + (char)i + "'"
                );
            System.exit(1);
        }

        return true;
    }
    
    private final int // Character classes
        CC_EOF      = 'Z',
        CC_WORD     = 'W',
        CC_COMMA    = ',',
        CC_BRACKET  = '[',
        CC_PAREN    = '(',
        CC_COLON    = ':',
        CC_NUMERIC  = '5',
        CC_SLASH    = '/',
        CC_WHITE    = '_';
    private int getCharClass(int c)
    {
        if(c == ' ' || c == '\t' || c == '\r' || c == '\n') return CC_WHITE;
        if(c == -1) return CC_EOF;
        if(c >= 'a' && c <= 'z' ||
           c >= 'A' && c <= 'Z' ||
           c == '_') return CC_WORD;
        if(c >= '0' && c <= '9' || c == '-' || c == '.') return CC_NUMERIC;
        if(c == ',') return CC_COMMA;
        if(c == ':') return CC_COLON;
        if(c == '/') return CC_SLASH;
        if(c == '[' || c == ']') return CC_BRACKET;
        if(c == '(' || c == ')') return CC_PAREN;
        
        System.out.println("public String getCharClass(int c): " +
                           "No class for char '" + (char)c + "'.");
        System.exit(0);
        return -1;
    }

    private static final int 
        SM_MAIN   = 0,
        SM_NAME   = 1,
        SM_NUMBER = 2;
    private int sm_State = SM_MAIN;
    private Token readNextToken(LineNumberReader lnr) throws IOException
    {
        for(;;)
        {
            // peek one character ahead
            int next;
            lnr.mark(1);
            next = lnr.read();
            lnr.reset();
            int cc = getCharClass(next);
            // System.out.print("(c:"+(char)next+", cc:" + (char)cc + ")");
            // System.out.print((char)next);

            // System.exit(0);

            switch(sm_State)
            {
            case SM_MAIN:
                switch(cc)
                {
                case CC_EOF    :
                    String tmp = currentToken;
                    currentToken = "";
                    Token eofT = new Token(Token.EOF, "<EOF>");
                    return (tmp == "")? null : eofT;
                    // break;
                case CC_WORD   :
                    sm_State = SM_NAME;
                    break;
                case CC_COMMA  :
                    lnr.skip(1);
                    return new Token(Token.COMMA, ",");
                    // break;
                case CC_BRACKET:
                    lnr.skip(1);
                    if(next == '[')
                        return new Token(Token.LBRACKET, "[");
                    else
                        return new Token(Token.RBRACKET, "]");
                    // break;
                case CC_PAREN:
                    lnr.skip(1);
                    if(next == '(')
                        return new Token(Token.LPAREN, "(");
                    else
                        return new Token(Token.RPAREN, ")");
                    // break
                case CC_COLON  :
                    lnr.skip(1);
                    return new Token(Token.COLON, ":");
                    // break;
                case CC_WHITE  :
                    lnr.skip(1);
                    break;
                case CC_NUMERIC:
                    sm_State = SM_NUMBER;
                    break;
                case CC_SLASH:
                    lnr.skip(1);
                    return new Token(Token.SLASH, "/");
                    // break;
                }
                break;
                
            case SM_NAME: 
                switch(cc)
                {
                case CC_WORD   :
                case CC_NUMERIC:
                    currentToken += (char)next;
                    lnr.skip(1);
                    break;
                default:
                    String tmp = currentToken;
                    currentToken = "";
                    sm_State = SM_MAIN;
                    return new Token(Token.WORD, tmp);
                    // break;
                }
                break;
                
            case SM_NUMBER: 
                switch(cc)
                {
                case CC_NUMERIC:
                    currentToken += (char)next;
                    lnr.skip(1);
                    break;
                default:
                    String tmp = currentToken;
                    currentToken = "";
                    sm_State = SM_MAIN;
                    return new Token(Token.NUMBER, tmp);
                    // break;
                }
                break;
            }
        }
        // return null;
    }

}
