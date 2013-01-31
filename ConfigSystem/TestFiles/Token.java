class Token
{
    public static final int 
        NONE      =   -1,
        WORD      =  'W',
        NUMBER    =  '#',
        LBRACKET  =  '[',
        RBRACKET  =  ']',
        LPAREN    =  '(',
        RPAREN    =  ')',
        COMMA     =  ',',
        COLON     =  ':',
        SLASH     =  '/',
        EOF       =  'Z';

    
    public int type = NONE;
    public String lexeme = "";

    public Token(int t, String l)
    {
        type = t;
        lexeme = l;
    }

    public String toString()
    {
        return "('" + (char)type + "':'" + lexeme + "')";
    }

    public static String typeString(int type)
    {
        switch(type)
        {
        case  -1: return "<NONE>";
        case 'W': return "string";
        case '#': return "double";
        case '[': return "["     ;
        case ']': return "]"     ;
        case '(': return "("     ;
        case ')': return ")"     ;
        case ',': return ","     ;
        case ':': return ":"     ;
        case '/': return "/"     ;
        case 'Z': return "<EOF>" ;
        }
        return "<ERROR>";
    }
}
