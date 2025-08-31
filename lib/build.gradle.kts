plugins {
    id("java-library")
}
java {
    sourceCompatibility = JavaVersion.VERSION_11
    targetCompatibility = JavaVersion.VERSION_11
}

dependencies {
    // https://mvnrepository.com/artifact/org.ejml/ejml-core
    implementation(libs.ejml.core)

    // https://mvnrepository.com/artifact/org.apache.commons/commons-statistics-distribution
    implementation(libs.commons.statistics.distribution)

    // https://mvnrepository.com/artifact/org.boofcv/boofcv-core
    implementation(libs.boofcv.core)
    // https://mvnrepository.com/artifact/org.boofcv/boofcv-swing
    implementation(libs.boofcv.swing)

    // https://mvnrepository.com/artifact/com.opencsv/opencsv
    implementation(libs.opencsv)

    // https://mvnrepository.com/artifact/org.apache.commons/commons-collections4
    implementation(libs.commons.collections4)

    // https://mvnrepository.com/artifact/org.yaml/snakeyaml
    implementation(libs.snakeyaml)

}