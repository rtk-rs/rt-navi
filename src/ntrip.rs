use thiserror::Error;

#[derive(Debug, Error, Copy, Clone)]
pub enum NTRIPParsing {}

#[derive(Debug, Clone)]
pub struct NTRIPInfos {
    pub host: String,
    pub port: u16,
    pub mount: String,
    pub username: Option<String>,
    pub password: Option<String>,
}

impl std::str::FromStr for NTRIPInfos {
    type Err = NTRIPParsing;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let mut host = String::default();
        let mut port = 0u16;
        let mut mount = String::default();
        let mut username = Option::<String>::None;
        let mut password = Option::<String>::None;

        for (index, item) in s.split('/').enumerate() {
            if index == 0 {
                for (subindex, subitem) in item.split(':').enumerate() {
                    if subindex == 0 {
                        host = subitem.to_string();
                    } else if subindex == 1 {
                        if let Ok(value) = subitem.trim().parse::<u16>() {
                            port = value;
                        }
                    } else {
                        panic!("Invalid --ntrip description. See rt-navi --help.");
                    }
                }
            } else if index == 1 {
                mount = item.trim().to_string();
            } else {
                for subitem in item.split(',') {
                    if subitem.starts_with("user=") {
                        username = Some(subitem.trim().to_string());
                    } else if subitem.starts_with("password=") {
                        password = Some(subitem.trim().to_string());
                    } else {
                        panic!("Invalid --ntrip description. See rt-navi --help.");
                    }
                }
            }
        }

        Ok(NTRIPInfos {
            username,
            password,
            host,
            mount,
            port,
        })
    }
}
