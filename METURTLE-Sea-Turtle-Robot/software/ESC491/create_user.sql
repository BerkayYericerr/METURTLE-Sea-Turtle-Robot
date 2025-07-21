CREATE TABLE users (
    id INTEGER NOT NULL PRIMARY KEY,
    username VARCHAR(50) NOT NULL,
    password VARCHAR(100) NOT NULL
);

-- Birkaç kullanıcı ekle
INSERT INTO users (id, username, password) VALUES (1, 'efe', '12345');
INSERT INTO users (id, username, password) VALUES (2, 'ali', 'abcdef');
