import ply.lex as lex

tokens = (
        "SYMBOL",
        "AND", "OR",
        "NOT",
        "TRUE",
        "ALL",
        "SOME",
        "LPAREN", "RPAREN")

t_SYMBOL = r"[b-df-z]+[a-z0-9]*"
t_TRUE   = r"1"
t_AND    = r"&&"
t_OR     = r"\|\|"
t_NOT    = r"!"
t_ALL    = r"a"
t_SOME   = r"e"
t_LPAREN = r"\("
t_RPAREN = r"\)"

t_ignore = " "


def t_error(t):
    print "Illegal character '%s'" % t.value[0]

def get_lexer():
    return lex.lex()
