use std::error::Error;
use std::fs;
use std::time;

pub struct EurocData {
    pub timestamp: time::Duration,
    pub img_name: String,
}

pub type EurocDataSet = Vec<EurocData>;

pub fn load_euroc_data(path: &str) -> Result<EurocDataSet, Box<dyn Error>> {
    let csv_path = format!("{}/data.csv", path);
    let img_path = format!("{}/data", path);
    // 打开文件
    let file = fs::File::open(csv_path)?;

    // 创建 CSV Reader
    let mut reader = csv::ReaderBuilder::new().has_headers(true).from_reader(file);

    let mut data_set = Vec::new();

    for record in reader.records() {
        let line = record?;
        let timestamp = line.get(0).unwrap();
        let img_name = line.get(1).unwrap().to_string();
        let timestamp = match timestamp.parse::<u64>() {
            Ok(timestamp) => timestamp,
            Err(e) => {
                println!("str: {}, Error: {}", timestamp, e);
                continue;
            },
        };
        
        let timestamp = time::Duration::from_nanos(timestamp);
        data_set.push(EurocData {
            timestamp,
            img_name: format!("{}/{}", img_path, img_name),
        });
    }

    Ok(data_set)
}


mod tests {
    #[test]
    fn test_read_csv() -> Result<(), Box<dyn std::error::Error>> {
        let path = "/home/zhang/Downloads/MH_01_easy/mav0/cam0";
        let data_set = super::load_euroc_data(path)?;
        for data in data_set {
            println!("{}, {}", data.timestamp.as_nanos(), data.img_name);
        }

        Ok(())
    }
}